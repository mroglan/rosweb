#ifndef ROSWEB_ROS_SUPPORTED_TYPES_H
#define ROSWEB_ROS_SUPPORTED_TYPES_H

#include <set>
#include <string>

namespace rosweb {
    namespace supported_ros_types {

        extern const std::set<std::string> msgs;

        void verify_is_supported_msg(const std::string& msg);
    }
}

#endif