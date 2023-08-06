#include <set>
#include <string>

#include "../include/supported_ros_types.h"
#include "../include/errors.h"

const std::set<std::string> rosweb::supported_ros_types::msgs{{
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/NavSatFix",
    "nav_msgs/msg/Odometry"
}};

void rosweb::supported_ros_types::verify_is_supported_msg(const std::string& msg) {
    if (rosweb::supported_ros_types::msgs.find(msg) == rosweb::supported_ros_types::msgs.end()) {
        throw rosweb::errors::request_error("Requested message type is not supported.");
    }
}