#include <string>
#include <unordered_map>

enum ActionName {
    NO_ACTION,
    STRAIGHT_FORWARD,
    STRAIGHT_BACKWARD,
    SLOW_LEFT_FORWARD,
    SLOW_RIGHT_FORWARD,
    SLOW_LEFT_BACKWARD,
    SLOW_RIGHT_BACKWARD,
    FAST_LEFT_FORWARD,
    FAST_RIGHT_FORWARD,
    FAST_LEFT_BACKWARD,
    FAST_RIGHT_BACKWARD
};

class Action {
    public:
        ActionName name;
        float angle;
        float throttle;
        constexpr Action(ActionName name, float angle, float throttle, int timestamp) : name(name),  angle(angle), throttle(throttle), timestamp(timestamp) {}
        static Action get_action_follow(float dx_ratio, float dy_ratio, float area_ratio);
        static Action get_action_waypoint(std::string command, float dx_ratio, float dy_ratio, float area_ratio);
        static std::string get_action_name(ActionName name);
        int timestamp;
};

class ActionAngle {
    public:
        static constexpr float NO_ACTION = 0.0;
        static constexpr float STRAIGHT_FORWARD = 0.0;
        static constexpr float STRAIGHT_BACKWARD = 0.0;
        static constexpr float SLOW_LEFT_FORWARD = 0.5;
        static constexpr float SLOW_RIGHT_FORWARD = -0.5;
        static constexpr float SLOW_LEFT_BACKWARD = 0.5;
        static constexpr float SLOW_RIGHT_BACKWARD = -0.5;
        static constexpr float FAST_LEFT_FORWARD = 1.0;
        static constexpr float FAST_RIGHT_FORWARD = -1.0;
        static constexpr float FAST_LEFT_BACKWARD = 1.0;
        static constexpr float FAST_RIGHT_BACKWARD = -1.0;
};

class ActionThrottle {
    public:
        static constexpr float NO_ACTION = 0.0;
        static constexpr float STRAIGHT_FORWARD = 0.55;
        static constexpr float STRAIGHT_BACKWARD = -0.5;
        static constexpr float SLOW_LEFT_FORWARD = 0.5;
        static constexpr float SLOW_RIGHT_FORWARD = 0.5;
        static constexpr float SLOW_LEFT_BACKWARD = -0.5;
        static constexpr float SLOW_RIGHT_BACKWARD = -0.5;
        static constexpr float FAST_LEFT_FORWARD = 0.5;
        static constexpr float FAST_RIGHT_FORWARD = 0.5;
        static constexpr float FAST_LEFT_BACKWARD = -0.5;
        static constexpr float FAST_RIGHT_BACKWARD = -0.5;
};

class ActionVals {
    public:
        static constexpr Action NO_ACTION = Action(ActionName::NO_ACTION, ActionAngle::NO_ACTION, ActionThrottle::NO_ACTION, 0);
        static constexpr Action STRAIGHT_FORWARD = Action(ActionName::STRAIGHT_FORWARD, ActionAngle::STRAIGHT_FORWARD, ActionThrottle::STRAIGHT_FORWARD, 0);
        static constexpr Action STRAIGHT_BACKWARD = Action(ActionName::STRAIGHT_BACKWARD, ActionAngle::STRAIGHT_BACKWARD, ActionThrottle::STRAIGHT_BACKWARD, 0);
        static constexpr Action SLOW_LEFT_FORWARD = Action(ActionName::SLOW_LEFT_FORWARD, ActionAngle::SLOW_LEFT_FORWARD, ActionThrottle::SLOW_LEFT_FORWARD, 0);
        static constexpr Action SLOW_RIGHT_FORWARD = Action(ActionName::SLOW_RIGHT_FORWARD, ActionAngle::SLOW_RIGHT_FORWARD, ActionThrottle::SLOW_RIGHT_FORWARD, 0);
        static constexpr Action SLOW_LEFT_BACKWARD = Action(ActionName::SLOW_LEFT_BACKWARD, ActionAngle::SLOW_LEFT_BACKWARD, ActionThrottle::SLOW_LEFT_BACKWARD, 0);
        static constexpr Action SLOW_RIGHT_BACKWARD = Action(ActionName::SLOW_RIGHT_BACKWARD, ActionAngle::SLOW_RIGHT_BACKWARD, ActionThrottle::SLOW_RIGHT_BACKWARD, 0);
               
};

// constexprants for determining the throttle from the area ratio
class ActionThrottleThreshold {
    public:
        // no action when area ratio is between 0.05 and 0.25
        static constexpr float NO_ACTION = 0.10;
        // go forward when area ratio <= 0.05
        static constexpr float FORWARD = 0.05;
        // go backwards when area ratio >= 0.20
        static constexpr float BACKWARD = 0.17;
};

class ActionAngleThreshold {
    public:
        static constexpr float NO_ACTION = 0.10;
        static constexpr float LEFT = 0.35;
        static constexpr float RIGHT = 0.65;
};