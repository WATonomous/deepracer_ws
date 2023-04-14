#include <string>
#include "action_space.hpp"



Action Action::get_action(float dx_ratio, float dy_ratio, float area_ratio) {
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