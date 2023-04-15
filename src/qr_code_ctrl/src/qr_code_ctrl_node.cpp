#include "rclcpp/rclcpp.hpp"
#include "qr_msgs/msg/qr_code_command.hpp"
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"
#include "action_space.hpp"
#include <deque>
#include <chrono>

class QRCodeCtrlNode : public rclcpp::Node
{
public:

    std::string drive_topic = "/cmdvel_to_servo_pkg/servo_msg";
    std::string qr_code_topic = "/qr_code_detector/qr_code_command";
    // 3 seconds to milliseconds
    int action_duration = std::chrono::milliseconds(2000).count();

    QRCodeCtrlNode() : Node("qr_code_ctrl_node")
    {   
        // Create default qos
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        qr_code_command_subscriber_ = this->create_subscription<qr_msgs::msg::QRCodeCommand>(
            qr_code_topic, default_qos, std::bind(&QRCodeCtrlNode::qr_code_command_callback, this, std::placeholders::_1));
        servo_ctrl_publisher_ = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(drive_topic, 10);

        // Initialize a timer to periodically publish the queued actions
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&QRCodeCtrlNode::publish_actions, this));
        // initialize wall timer with default QOS



    }

private:
    void qr_code_command_callback(const qr_msgs::msg::QRCodeCommand::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received QRCodeCommand: throttle=%f, turn_angle=%f, area_ratio=%f", msg->throttle, msg->turn_angle, msg->area_ratio);
        deepracer_interfaces_pkg::msg::ServoCtrlMsg servo_ctrl_msg;

        // Map QRCodeCommand values to ServoCtrlMsg
        // servo_ctrl_msg.throttle = msg->throttle;
        // servo_ctrl_msg.angle = msg->turn_angle;
        Action action = ActionVals::NO_ACTION;
        if (msg->command == "NO_ACTION") {
            action = ActionVals::NO_ACTION;
        } else {
            // For Follow the Leader
            // action = Action::get_action_follow(msg->center_x_ratio, msg->center_y_ratio, msg->area_ratio);

            // For Waypoint
            action = Action::get_action_waypoint(msg->command, msg->center_x_ratio, msg->center_y_ratio, msg->area_ratio);
            // set the action timestamp
            action.timestamp = msg->timestamp;
        }
        
        // Add action to queue if it is different from the current action
        if (action_queue_.empty() || action_queue_.front().name != action.name) {
            action_queue_.push_back(action);
        }

        // // Publish ServoCtrlMsg
        // servo_ctrl_publisher_->publish(servo_ctrl_msg);
    }

    void publish_actions()
    {
        // Publish the queued actions
        if (!action_queue_.empty()) {
            auto action = action_queue_.front();
            int now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            int age = now - action.timestamp;
            RCLCPP_INFO(this->get_logger(), "Age: %d, now %d, timestamp %d, duration %d", age, now, action.timestamp, action_duration);

            // Remove action if too old
            if (age >= action_duration) {
                RCLCPP_INFO(this->get_logger(), "Removing action");
                action_queue_.pop_front();
            } else {
                RCLCPP_INFO(this->get_logger(), "Publishing action");


                deepracer_interfaces_pkg::msg::ServoCtrlMsg servo_ctrl_msg;

                if (action.name != ActionName::NO_ACTION) {
                    RCLCPP_INFO(this->get_logger(), "Publishing Action: %s Throttle = %f, Angle = %f", Action::get_action_name(action.name).c_str(), action.throttle, action.angle);
                }

                servo_ctrl_msg.throttle = action.throttle;
                servo_ctrl_msg.angle = action.angle;

                servo_ctrl_publisher_->publish(servo_ctrl_msg);
            }
        }

        // If queue is empty, publish a stop action
        // This is because the car will continue to drive if the last action was a drive action
        if (action_queue_.empty()) {
            deepracer_interfaces_pkg::msg::ServoCtrlMsg servo_ctrl_msg;
            servo_ctrl_msg.throttle = 0.0;
            servo_ctrl_msg.angle = 0.0;
            servo_ctrl_publisher_->publish(servo_ctrl_msg);
        }
    }

    rclcpp::Subscription<qr_msgs::msg::QRCodeCommand>::SharedPtr qr_code_command_subscriber_;
    rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr servo_ctrl_publisher_;
    std::deque<Action> action_queue_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QRCodeCtrlNode>());
    rclcpp::shutdown();
    return 0;
}
