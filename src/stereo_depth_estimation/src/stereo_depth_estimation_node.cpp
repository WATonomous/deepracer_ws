#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include "deepracer_interfaces_pkg/msg/camera_msg.hpp"

#include <iostream>
#include <chrono>

#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>


using namespace std::literals::chrono_literals;

#include <cmath>
#define _USE_MATH_DEFINES

using std::placeholders::_1;

class StereoDepthEstimationNode : public rclcpp::Node {

    std::string subscriber_topic = "/camera_pkg/video_mjpeg";
    std::string publisher_topic = "/stereo_depth_map";

    // Create ROS subscribers and publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Subscription<deepracer_interfaces_pkg::msg::CameraMsg>::SharedPtr subscription;

    cv::Ptr<cv::StereoBM> stereo;

    // Params for stereo
    int n_disparities = 16;
    int window_size = 7;

public:
    StereoDepthEstimationNode() : Node("stereo_depth_estimation_node")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // Create ROS subscribers and publishers
        publisher = this->create_publisher<sensor_msgs::msg::Image>(this->publisher_topic, default_qos);
        subscription = this->create_subscription<deepracer_interfaces_pkg::msg::CameraMsg>(this->subscriber_topic, default_qos, std::bind(&StereoDepthEstimationNode::subscriber_callback, this, _1));
        stereo = cv::StereoBM::create(n_disparities, window_size);
    }

private:
    sensor_msgs::msg::Image::SharedPtr get_depth_map(const deepracer_interfaces_pkg::msg::CameraMsg::ConstSharedPtr& msg) {

        cv::Mat imgL_gray, imgR_gray;
        cv::Mat disp, disparity;

        cv_bridge::CvImagePtr imgL_ptr, imgR_ptr;

        try
        {
            imgL_ptr = cv_bridge::toCvCopy(msg->images[0], sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "cv_bridge exception: " << e.what());
            return nullptr;
        }

        try
        {
            imgR_ptr = cv_bridge::toCvCopy(msg->images[1], sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "cv_bridge exception: " << e.what());
            return nullptr;
        }

        // convert to grayscale
        cv::cvtColor(imgL_ptr->image, imgL_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(imgR_ptr->image, imgR_gray, cv::COLOR_BGR2GRAY);

        (this->stereo)->compute(imgL_gray,imgR_gray,disp);


        // Convert cv image to ros2 message
        auto pub_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", disp)
            .toImageMsg();

        return pub_msg;
    }
    void subscriber_callback(const deepracer_interfaces_pkg::msg::CameraMsg::ConstSharedPtr camera_msg) {
        // For stereo camera setup, CameraMsg is expected to contain an array of two images (left and right)
        assert(camera_msg->images.size() == 2);

        std::vector<sensor_msgs::msg::Image> images = camera_msg->images;

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Received camera msg");

        auto pub_msg = this->get_depth_map(camera_msg);

        // Publish the depth map
        if (pub_msg) {
            publisher->publish(*pub_msg.get());
        }
    }
};

int main(int argc, char ** argv) {
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Starting stereo depth estimation node..." );
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoDepthEstimationNode>());
    rclcpp::shutdown();
    return 0;
}