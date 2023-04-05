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
                msg.command = command;
                msg.throttle = throttle;
                msg.turn_angle = turn_angle;
                command_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "QR Code detected with command: %s, throttle: %f, turn_angle: %f", command.c_str(), throttle, turn_angle);
            }
            else
            {
                // Log error wrong format
                RCLCPP_ERROR(this->get_logger(), "QR Code detected with wrong format, expected format '{command} {throttle} {turn_angle}' but got: %s", symbol->get_data().c_str());
            }
            
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