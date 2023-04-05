#include "rclcpp/rclcpp.hpp"
#include "qr_msgs/msg/qr_code_command.hpp"
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"

class QRCodeCtrlNode : public rclcpp::Node
{
public:

    std::string drive_topic = "/ctrl_pkg/servo_msg";
    
    QRCodeCtrlNode() : Node("qr_code_ctrl_node")
    {   
        // Create default qos
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        qr_code_command_subscriber_ = this->create_subscription<qr_msgs::msg::QRCodeCommand>(
            "/qr_code_detector/qr_code_command", default_qos, std::bind(&QRCodeCtrlNode::qr_code_command_callback, this, std::placeholders::_1));
        servo_ctrl_publisher_ = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(drive_topic, 10);
    }

private:
    void qr_code_command_callback(const qr_msgs::msg::QRCodeCommand::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received QRCodeCommand: throttle=%f, turn_angle=%f", msg->throttle, msg->turn_angle);
        deepracer_interfaces_pkg::msg::ServoCtrlMsg servo_ctrl_msg;

        // Map QRCodeCommand values to ServoCtrlMsg
        servo_ctrl_msg.throttle = msg->throttle;
        servo_ctrl_msg.angle = msg->turn_angle;

        // Publish ServoCtrlMsg
        servo_ctrl_publisher_->publish(servo_ctrl_msg);
    }

    rclcpp::Subscription<qr_msgs::msg::QRCodeCommand>::SharedPtr qr_code_command_subscriber_;
    rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servo_ctrl_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
