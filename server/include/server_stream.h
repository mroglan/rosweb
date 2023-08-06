#ifndef ROSWEB_SERVER_STREAM_H
#define ROSWEB_SERVER_STREAM_H

#include <memory>
#include <map>
#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "./json.hpp"

namespace rosweb {
    class server_stream {
        public:

            void add_msg(const std::string& topic_name,
                const sensor_msgs::msg::Image::SharedPtr msg);
            
            void add_msg(const std::string& topic_name,
                const sensor_msgs::msg::NavSatFix::SharedPtr msg);
            
            void add_msg(const std::string& topic_name,
                const nav_msgs::msg::Odometry::SharedPtr msg);
            
            void clear();

            void reset_timestamps();

            std::string stringify() const;
        
        private:
            nlohmann::json_abi_v3_11_2::json m_data;

            std::map<std::string, std::uint32_t> m_timestamps;
    };
}

#endif