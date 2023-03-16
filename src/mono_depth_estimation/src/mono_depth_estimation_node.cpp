
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "deepracer_interfaces_pkg/msg/camera_msg.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using std::placeholders::_1;


int clip(int n, int lower, int upper) {
    return std::max(lower, std::min(n, upper));
}


std::vector<std::string> getOutputsNames(const cv::dnn::Net& net)
{
    static std::vector<std::string> names;
    if (names.empty()) {
        std::vector<int32_t> out_layers = net.getUnconnectedOutLayers();
        std::vector<std::string> layers_names = net.getLayerNames();
        names.resize(out_layers.size());
        for (size_t i = 0; i < out_layers.size(); ++i) {
            names[i] = layers_names[out_layers[i] - 1];
        }
    }
    return names;
}



class MonoDepthEstimationNode : public rclcpp::Node {

    std::string subscriber_topic = "/camera_pkg/video_mjpeg";
    std::string publisher_topic = "/mono_depth_map";

    // Create ROS subscribers and publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Subscription<deepracer_interfaces_pkg::msg::CameraMsg>::SharedPtr subscription;

    // model
    cv::dnn::Net net;

public:
    MonoDepthEstimationNode() : Node("stereo_depth_estimation_node")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // Create ROS subscribers and publishers
        publisher = this->create_publisher<sensor_msgs::msg::Image>(this->publisher_topic, default_qos);
        subscription = this->create_subscription<deepracer_interfaces_pkg::msg::CameraMsg>(this->subscriber_topic, default_qos, std::bind(&MonoDepthEstimationNode::subscriber_callback, this, _1));

        // load model
        std::string file_path = "/perception_models/";

        // Read Network
        std::string model = "model-f6b98070.onnx"; // MiDaS v2.1 Large
        
        //string model = "model-small.onnx"; // MiDaS v2.1 Small
        

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "loading model " << model << "...");

        // Read in the neural network from the files
        net = cv::dnn::readNet(file_path+model);

        if (net.empty())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "unable to read dnn net");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "successfully loaded model");


    }

private:
    sensor_msgs::msg::Image::SharedPtr get_depth_map(const deepracer_interfaces_pkg::msg::CameraMsg::ConstSharedPtr& msg) {


    // Run on either CPU or GPU
    // net.setPreferableBackend(DNN_BACKEND_CUDA);
    // net.setPreferableTarget(DNN_TARGET_CUDA);

 
        // Load in an image
        cv::Mat image;
        cv_bridge::CvImagePtr imgPtr;


        try
        {
            imgPtr = cv_bridge::toCvCopy(msg->images[0], sensor_msgs::image_encodings::BGR8);
            image = imgPtr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "cv_bridge exception: " << e.what());
            return nullptr;
        }


        int image_width = image.rows;
        int image_height = image.cols;

        auto start = cv::getTickCount();


        // Create Blob from Input Image
        // MiDaS v2.1 Large ( Scale : 1 / 255, Size : 384 x 384, Mean Subtraction : ( 123.675, 116.28, 103.53 ), Channels Order : RGB )
        cv::Mat blob = cv::dnn::blobFromImage(image, 1 / 255.f, cv::Size(384, 384), cv::Scalar(123.675, 116.28, 103.53), true, false);
        // MiDaS v2.1 Small ( Scale : 1 / 255, Size : 256 x 256, Mean Subtraction : ( 123.675, 116.28, 103.53 ), Channels Order : RGB )
        // cv::Mat blob = cv::dnn::blobFromImage(image, 1 / 255.f, cv::Size(256, 256), cv::Scalar(123.675, 116.28, 103.53), true, false);

        // Set the blob to be input to the neural network
        net.setInput(blob);

        // Forward pass of the blob through the neural network to get the predictions
        cv::Mat output = net.forward(getOutputsNames(net)[0]);

        // Convert Size to 384x384 from 1x384x384
        const std::vector<int32_t> size = { output.size[1], output.size[2] };
        output = cv::Mat(static_cast<int32_t>(size.size()), &size[0], CV_32F, output.ptr<float>());

        // Resize Output Image to Input Image Size
        cv::resize(output, output, image.size());

        // Visualize Output Image

        double min, max;
        cv::minMaxLoc(output, &min, &max);
        const double range = max - min;

        // 1. Normalize ( 0.0 - 1.0 )
        output.convertTo(output, CV_32F, 1.0 / range, -(min / range));

        // 2. Scaling ( 0 - 255 )
        output.convertTo(output, CV_8U, 255.0);



        auto end = cv::getTickCount();
        auto totalTime = (end - start) / cv::getTickFrequency();
        auto fps = 1 / totalTime;

	cv::imwrite("/tmp/depth.png", output);


        // Convert cv image to ros2 message
        auto pub_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", output)
            .toImageMsg();

        return pub_msg;
    }
    void subscriber_callback(const deepracer_interfaces_pkg::msg::CameraMsg::ConstSharedPtr camera_msg) {
        // For mono camera setup, CameraMsg is expected to contain at least one image (left or right)
        assert(camera_msg->images.size() > 0);

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
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Starting mono depth estimation node..." );
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonoDepthEstimationNode>());
    rclcpp::shutdown();
    return 0;
}



