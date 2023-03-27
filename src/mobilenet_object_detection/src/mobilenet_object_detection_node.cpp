
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <vector>
#include <unordered_map>
#include <sstream>

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using std::placeholders::_1;

struct Detection
{
    int class_id;
    float confidence;
    cv::Rect box;
};

std::vector<std::string> read_classes(std::string class_path, std::vector<std::string> &classes)
{
    std::ifstream ifs(class_path);
    std::string line;
    while (getline(ifs, line))
    {
        classes.push_back(line);
    }
    // for (auto s : classes)
    // {
    //     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), s);
    // }
    return classes;
}

class ObjectDetectionNode : public rclcpp::Node
{
    std::string model_path;
    std::string model_config_path;
    std::string class_path;
    std::vector<std::string> classes;

    std::string subscriber_topic;
    std::string publisher_topic;

    // Create ROS subscribers and publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription;

    // model
    cv::dnn::Net net;

public:
    ObjectDetectionNode() : Node("mobilenet_object_detection_node")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // load params from launch file
        // Get model from https://github.com/opencv/opencv/wiki/TensorFlow-Object-Detection-API
        this->declare_parameter("model_path");
        this->declare_parameter("model_config_path");
        this->declare_parameter("classes_txt_path");
        this->declare_parameter("publisher_topic");
        this->declare_parameter("subscriber_topic");

        this->get_parameter("model_path", model_path);
        this->get_parameter("model_config_path", model_config_path);
        this->get_parameter("classes_txt_path", class_path);
        this->get_parameter("publisher_topic", publisher_topic);
        this->get_parameter("subscriber_topic", subscriber_topic);

        // Read in the classes
        read_classes(class_path, classes);

        // Create ROS subscribers and publishers
        publisher = this->create_publisher<sensor_msgs::msg::Image>(this->publisher_topic, default_qos);
        subscription = this->create_subscription<sensor_msgs::msg::Image>(this->subscriber_topic, default_qos, std::bind(&ObjectDetectionNode::subscriber_callback, this, _1));

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "loading model " << model_path << "...");

        // Read in the neural network from the files
        // config is optional for onnx, required for OpenVINO
        if (model_config_path != "")
        {
            net = cv::dnn::readNet(model_path, model_config_path);
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "loading model config " << model_config_path << "...");
        }
        else
        {
            net = cv::dnn::readNet(model_path);
        }

        if (net.empty())
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "unable to read dnn net");
        }

        // // required for onnx and opencv 4.7, or else inference will return error
        // net.enableWinograd(false);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "successfully loaded model");
    }

private:
    cv::Mat preprocess_image(const cv::Mat &source)
    {
        int col = source.cols;
        int row = source.rows;
        int _max = MAX(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        source.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }

    void get_detections(cv::Mat &image, std::vector<Detection> &detections)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Prepare input blob
        cv::Size inputSize(300, 300);
        cv::Scalar mean(127.5, 127.5, 127.5);
        double scale = 1.0 / 127.5;
        cv::resize(image, image, inputSize);
        // cv::Mat blob = cv::dnn::blobFromImage(image, scale, inputSize, mean, false, false);
        cv::Mat blob = cv::dnn::blobFromImage(image, 1.0 / 127.5, inputSize, mean, true, false);

        // Set input blob
        net.setInput(blob);
        std::vector<cv::Mat> outs;
        std::vector<std::string> outNames = net.getUnconnectedOutLayersNames();
        // Forward pass
        net.forward(outs, outNames);

        for (size_t k = 0; k < outs.size(); k++)
        {
            float *data = (float *)outs[k].data;
            for (size_t i = 0; i < outs[k].total(); i += 7)
            {
                float confidence = data[i + 2];
                int classId = (int)data[i + 1] - 1;
                if (confidence > 0.4)
                {
                    float left = data[i + 3] * image.cols;
                    float top = data[i + 4] * image.rows;
                    float right = data[i + 5] * image.cols;
                    float bottom = data[i + 6] * image.rows;

                    // Add 1 because cv::Rect() defines the boundary as left and top are inclusive,
                    //  and as right and bottom are exclusive?
                    float width = right - left + 1;
                    float height = bottom - top + 1;

                    // Add detection to the list

                    // log class and confidence
                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "class id: " << classId << " class: "
                                                                                  << classes[classId] << " confidence: " << confidence);

                    cv::Rect rect(left, top, width, height);
                    detections.push_back(Detection{classId, confidence, rect});
                }
            }
        }

        // Compute fps
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        double fps = 1000.0 / duration.count();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "inference fps: " << fps);
    }

    void publish_detections(cv::Mat &src_image, const std::vector<Detection> &detections)
    {
        // Load in an image
        cv::Mat image = src_image.clone();

        // Draw boxes on image
        for (size_t i = 0; i < detections.size(); i++)
        {
            // Draw box
            cv::rectangle(image, detections[i].box, cv::Scalar(255, 0, 0), 2);

            // Draw label
            std::stringstream ss;
            ss << classes[detections[i].class_id] << ": " << std::fixed << std::setprecision(2) << detections[i].confidence;
            std::string label = ss.str();

            int baseline;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
            cv::rectangle(image, cv::Point(detections[i].box.x, detections[i].box.y - label_size.height - baseline),
                          cv::Point(detections[i].box.x + label_size.width, detections[i].box.y), cv::Scalar(0, 255, 0), cv::FILLED);

            cv::putText(image, label, cv::Point(detections[i].box.x, detections[i].box.y - baseline),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), label << std::endl);
        }

        // Convert cv image to ros image
        auto pub_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image)
                           .toImageMsg();

        // Publish image
        publisher->publish(*pub_msg.get());
    }

    void subscriber_callback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        // Load in the image
        cv::Mat image;
        cv_bridge::CvImagePtr imgPtr;

        // Read in the image
        try
        {
            imgPtr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            image = imgPtr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "cv_bridge exception: " << e.what());
        }

        std::vector<Detection> detections;
        this->get_detections(image, detections);
        this->publish_detections(image, detections);
    }
};

int main(int argc, char **argv)
{
    RCLCPP_INFO(rclcpp::get_logger("logger"), "Starting object detection node...");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
