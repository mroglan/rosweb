#ifndef ROSWEB_ROS_SESSION_H
#define ROSWEB_ROS_SESSION_H

#include <memory>
#include <iostream>
#include <map>
#include <set>
#include <boost/variant.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "./server_stream.h"

namespace rosweb {
    class bridge;
    namespace client_requests {
        class client_request_handler;
    }
    namespace server_responses {
        class standard;
    }
}

namespace rosweb {
    class ros_session : public rclcpp::Node {
        public:
            ros_session(std::shared_ptr<rosweb::bridge> bridge);
        
        private:

            struct sub_wrapper {
                std::map<std::string, std::string> types;
                std::set<std::string> paused;

                std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs;
                std::map<std::string, sensor_msgs::msg::Image::SharedPtr> image_data;
            } m_sub_wrapper;

            std::shared_ptr<rosweb::bridge> m_bridge;

            rclcpp::TimerBase::SharedPtr m_timer;

            std::unique_ptr<rosweb::server_stream> m_stream;

            void timer_callback();

            void handle_new_request(rosweb::server_responses::standard*& res);

            void create_subscriber(
                const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
                rosweb::server_responses::standard*& res
            );
            
            void destroy_subscriber(
                const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
                rosweb::server_responses::standard*& res
            );

            void change_subscriber(
                const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
                rosweb::server_responses::standard*& res
            );

            void toggle_pause_subscriber(
                const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
                rosweb::server_responses::standard*& res
            );

            void create_sub_helper(const std::string& topic_name, const std::string& msg_type);

            void destroy_sub_helper(const std::string& topic_name, const std::string& msg_type);

            void bagged_image_to_video(
                const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
                rosweb::server_responses::standard*& res
            );
    };
}

#endif