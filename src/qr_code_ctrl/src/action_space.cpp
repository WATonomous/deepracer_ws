#include <string>
#include "action_space.hpp"
#include "qr_msgs/msg/qr_code_command.hpp"

// Goal: Follow the QR code and keep a distance (follow the leader)
// Computes the action based on the QR code size ratio to the screen, as well as position
// If size ratio is too big, move backwards, if too small, move forward
// If position of QR code is too far left, turn left, if too far right, turn right
Action Action::get_action_follow(float dx_ratio, float dy_ratio, float area_ratio) {
    // special case: area_ratio is -1.0 (no qr code detected)
    if (area_ratio == -1.0) {
        return ActionVals::NO_ACTION;
    }
    
    if (area_ratio < ActionThrottleThreshold::FORWARD) {
        if (dx_ratio < ActionAngleThreshold::LEFT) {
            return ActionVals::SLOW_LEFT_FORWARD;
        } else if (dx_ratio > ActionAngleThreshold::RIGHT) {
            return ActionVals::SLOW_RIGHT_FORWARD;
        } else {
            return ActionVals::STRAIGHT_FORWARD;
        }
    } else if (area_ratio > ActionThrottleThreshold::BACKWARD) {
        if (dx_ratio < ActionAngleThreshold::LEFT) {
            return ActionVals::SLOW_LEFT_BACKWARD;
        } else if (dx_ratio > ActionAngleThreshold::RIGHT) {
            return ActionVals::SLOW_RIGHT_BACKWARD;
        } else {
            return ActionVals::STRAIGHT_BACKWARD;
        }
    // No action
    } else {
            return ActionVals::NO_ACTION;
    }
}

// Goal: Treat the QR code as a waypoint on a path
Action Action::get_action_waypoint(std::string command, float dx_ratio, float dy_ratio, float area_ratio) {
    // special case: area_ratio is -1.0 (no qr code detected)
    if (area_ratio == -1.0) {
        return ActionVals::NO_ACTION;
    }
        
    if (command == "stop") {
        return ActionVals::NO_ACTION;
    } else if (command == "forward") {
        return ActionVals::STRAIGHT_FORWARD;
    } else if (command == "left") {
        return ActionVals::SLOW_LEFT_FORWARD;
    } else if (command == "right") {
        return ActionVals::SLOW_RIGHT_FORWARD;
    } else {
            return ActionVals::NO_ACTION;
    }
}

std::string Action::get_action_name(ActionName name) {
    if (name == ActionName::NO_ACTION) {
        return "NO_ACTION";
    } else if (name == ActionName::STRAIGHT_FORWARD) {
        return "STRAIGHT_FORWARD";
    } else if (name == ActionName::STRAIGHT_BACKWARD) {
        return "STRAIGHT_BACKWARD";
    } else if (name == ActionName::SLOW_LEFT_FORWARD) {
        return "SLOW_LEFT_FORWARD";
    } else if (name == ActionName::SLOW_RIGHT_FORWARD) {
        return "SLOW_RIGHT_FORWARD";
    } else if (name == ActionName::SLOW_LEFT_BACKWARD) {
        return "SLOW_LEFT_BACKWARD";
    } else if (name == ActionName::SLOW_RIGHT_BACKWARD) {
        return "SLOW_RIGHT_BACKWARD";
    } else if (name == ActionName::FAST_LEFT_FORWARD) {
        return "FAST_LEFT_FORWARD";
    } else if (name == ActionName::FAST_RIGHT_FORWARD) {
        return "FAST_RIGHT_FORWARD";
    } else if (name == ActionName::FAST_LEFT_BACKWARD) {
        return "FAST_LEFT_BACKWARD";
    } else if (name == ActionName::FAST_RIGHT_BACKWARD) {
        return "FAST_RIGHT_BACKWARD";
    } else {
        return "UNKNOWN_ACTION";
    }
}