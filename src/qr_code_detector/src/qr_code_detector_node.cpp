#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include "qr_msgs/msg/qr_code_command.hpp"
#include <sstream>
#include <zbar.h>

using std::placeholders::_1;

class QRCodeDetectorNode : public rclcpp::Node
{
public:
    // Constructor
    QRCodeDetectorNode() : Node("qr_code_detector_node")
    {
        // Create default qos
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera_pkg/display_mjpeg", default_qos, std::bind(&QRCodeDetectorNode::image_callback, this, _1));

        // Initialize qr_code_detector
        qr_code_detector_ = cv::QRCodeDetector();

        command_publisher_ = this->create_publisher<qr_msgs::msg::QRCodeCommand>("qr_code_command", 10);
    }

private:
    // Private members
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<qr_msgs::msg::QRCodeCommand>::SharedPtr command_publisher_;

    cv::QRCodeDetector qr_code_detector_;

    // Number of consecutive frames without QR code
    int num_consecutive_no_qr_code = 0;
    int NUM_CONSECUTIVE_NO_QR_CODE_THRESHOLD = 2;

    int quad_area(const cv::Point &p1, const cv::Point &p2, const cv::Point &p3, const cv::Point &p4)
    {
        return abs((p1.x - p2.x) * (p1.y + p2.y) + (p2.x - p3.x) * (p2.y + p3.y) + (p3.x - p4.x) * (p3.y + p4.y) + (p4.x - p1.x) * (p4.y + p1.y));
    }

    // Callback function
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS Image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Process image and detect QR codes
        detect_qr_codes(cv_ptr->image);
    }

    // QR code detection function
    void detect_qr_codes(const cv::Mat &image)
    {
        // Convert image to grayscale
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

        // Create a ZBar Image Scanner
        zbar::ImageScanner scanner;
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);

        // Create a ZBar Image from the OpenCV Mat
        unsigned int width = gray_image.cols;
        unsigned int height = gray_image.rows;
        uchar *raw_data = gray_image.data;
        zbar::Image zbar_image(width, height, "Y800", raw_data, width * height);

        // Scan for QR codes
        int num_codes = scanner.scan(zbar_image);

        // Iterate through detected symbols
        for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin(); symbol != zbar_image.symbol_end(); ++symbol)
        {
            std::string payload_str = symbol->get_data();
            std::istringstream payload_stream(payload_str);

            qr_msgs::msg::QRCodeCommand msg;
            std::string command;
            float throttle, turn_angle;

            // Try reading in command with <<
            if (payload_stream >> command >> throttle >> turn_angle)
            {
                // Collect the bounding box points
                std::vector<cv::Point> points;
                for (int i = 0; i < symbol->get_location_size(); i++)
                {
                    points.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));
                }
                // Expect 4 points
                if (points.size() != 4)
                {
                    RCLCPP_ERROR(this->get_logger(), "QR Code detected with wrong number of points, expected 4 but got: %d", points.size());
                    return;
                }

                // compute relative size of the QR code as the ratio of the area of the bounding box to the area of the image
                float qr_code_area = quad_area(points[0], points[1], points[2], points[3]);
                float image_area = image.rows * image.cols;
                float qr_code_area_ratio = qr_code_area / image_area;

                // RCLCPP_INFO(this->get_logger(), "QR Code detected with area ratio: %f", qr_code_area_ratio);
                
                // Compute dx and dy ratios
                // Compute dx as the mean of the x coordinates of the top and bottom points
                float dx = (points[0].x + points[3].x) / 2.0;
                // Compute dy as the mean of the y coordinates of the left and right points
                float dy = (points[0].y + points[1].y) / 2.0;

                // Normalize dx and dy between 0 and 1.0
                dx = dx / image.cols;
                dy = dy / image.rows;

                
                msg.command = command;
                msg.throttle = throttle;
                msg.turn_angle = turn_angle;
                msg.area_ratio = qr_code_area_ratio;
                msg.center_x_ratio = dx;
                msg.center_y_ratio = dy;

                command_publisher_->publish(msg);
                // RCLCPP_INFO(this->get_logger(), "QR Code detected with points: (%d, %d), (%d, %d), (%d, %d), (%d, %d)", points[0].x, points[0].y, points[1].x, points[1].y, points[2].x, points[2].y, points[3].x, points[3].y);
                // RCLCPP_INFO(this->get_logger(), "QR Code detected with command: %s, throttle: %f, turn_angle: %f", command.c_str(), throttle, dx);
            }
            else
            {
                // Log error wrong format
                RCLCPP_ERROR(this->get_logger(), "QR Code detected with wrong format, expected format '{command} {throttle} {turn_angle}' but got: %s", symbol->get_data().c_str());
            }
        }
        // if no QR codes detected, publish a message with empty command
        if (num_codes == 0)
        {
            num_consecutive_no_qr_code++;
            qr_msgs::msg::QRCodeCommand msg;
            msg.command = "NO_ACTION";
            msg.throttle = 0.0;
            msg.turn_angle = 0.0;
            msg.area_ratio = -1.0;
            msg.center_x_ratio = 0.0;
            msg.center_y_ratio = 0.0;
            if (num_consecutive_no_qr_code > NUM_CONSECUTIVE_NO_QR_CODE_THRESHOLD) {
                command_publisher_->publish(msg);
            }
        } else {
            num_consecutive_no_qr_code = 0;
        }

        // Clean up
        zbar_image.set_data(nullptr, 0);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeDetectorNode>());
    rclcpp::shutdown();
    return 0;
}