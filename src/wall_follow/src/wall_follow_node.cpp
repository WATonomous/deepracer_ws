#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "deepracer_interfaces_pkg/msg/servo_ctrl_msg.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Starting wall follow node...\n");
        // Create ROS subscribers and publishers
        publisher_ = this->create_publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>(this->drive_topic, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Publishing drive topic on " + drive_topic);
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Subscribing lidar topic on " + lidarscan_topic);
    }

private:
    // PID CONTROL PARAMS
    double kp = 3;
    double ki = 0;
    double kd = 0.1;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;
    double start_t = -1;
    double curr_t = 0.0;
    double prev_t = 0.0;
    // Distance from the wall in meters
    double desired_distance = 0.50;
    
    // Topics
    std::string lidarscan_topic = "/rplidar_ros/scan";
    std::string drive_topic = "/ctrl_pkg/servo_msg";

    /// Create ROS subscribers and publishers
    rclcpp::Publisher<deepracer_interfaces_pkg::msg::ServoCtrlMsg>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

    double get_range(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR in radians

        Returns:
            range: range measurement in meters at the given angle
        */

        assert(angle >= scan_msg->angle_min && angle <= scan_msg->angle_max); // Angle must be within range
        int i = (angle - scan_msg->angle_min) / (scan_msg->angle_increment); // index i of closest angle
        if (std::isnan(scan_msg->ranges[i]) || scan_msg->ranges[i] > scan_msg->range_max) return scan_msg->range_max; // In case of NaNs and infinity, just return the maximum of the scan message
        return scan_msg->ranges[i];
    }
    
    double to_radians(double theta) {
        return M_PI * theta / 180.0;

    }
    
    double to_degrees(double theta) {
        return theta * 180.0 / M_PI;
    }

    void get_error(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double a = get_range(scan_msg, to_radians(50.0));
        double b = get_range(scan_msg, to_radians(90.0)); // 0 degrees is in front of the card.
        double theta = to_radians(40.0); // 90.0 - 50.0 = 40.0 degrees
        double alpha = std::atan((a * std::cos(theta) - b)/(a * std::sin(theta)));
        double D_t = b*std::cos(alpha);
        // double D_t_1 =  D_t + dist * std::sin(alpha);

        this->prev_error = this->error;
        this->error = dist - D_t;
        this->integral += this->error;
        this->prev_t = this->curr_t;
        this->curr_t = (double) scan_msg->header.stamp.nanosec * (double)10e-9 + (double) scan_msg->header.stamp.sec;
        if (this->start_t == 0.0) {
            this->start_t = this->curr_t;
        }
    }

    void pid_control()
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // Use kp, ki & kd to implement a PID controller
        if (this->prev_t == 0.0) return;
        angle = this->kp * this->error + this->ki * this->integral * (this->curr_t - this->start_t) + this->kd * (this->error - this->prev_error)/(this->curr_t - this->prev_t);

        auto drive_msg = deepracer_interfaces_pkg::msg::ServoCtrlMsg();
        // Fill in drive message and publish
        drive_msg.angle = angle;
        
        // // TODO: Deepracer messages use steering ratio, not absolute desired angle (https://github.com/aws-deepracer/aws-deepracer-interfaces-pkg/blob/main/deepracer_interfaces_pkg/msg/ServoCtrlMsg.msg)
        // // Check if this logic is still OK or if we need to adjust

        // // We go slower if we need to a large steering angle correction
        // if (std::abs(drive_msg.angle) >= this->to_radians(0) && std::abs(drive_msg.angle) < this->to_radians(10)) {
        //     drive_msg.throttle = 1.5;
        // } else if (std::abs(drive_msg.angle) >= this->to_radians(10) && std::abs(drive_msg.angle) < this->to_radians(20)) {
        //     drive_msg.throttle = 1.0;
        // } else {
        //     drive_msg.throttle = 0.5;
        // }

        drive_msg.throttle = 0.5;
        
        RCLCPP_INFO(rclcpp::get_logger("logger"), "Angle " + std::to_string(angle) + " Throttle " + std::to_string(drive_msg.throttle) );
        this->publisher_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        get_error(scan_msg, desired_distance); 
        pid_control();

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
