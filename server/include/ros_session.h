#ifndef ROSWEB_ROS_SESSION_H
#define ROSWEB_ROS_SESSION_H

#include <memory>
#include <iostream>
#include <map>
#include <boost/variant.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

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

            template<typename T>
            class sub_wrapper {
                public:

                    sub_wrapper(rclcpp::Node* node, const std::string& topic_name,
                        const std::string& msg_type)
                        : m_topic_name{topic_name}, m_msg_type{msg_type} {
                        m_sub = node->create_subscription<T>(
                            topic_name, 10, 
                            std::bind(&sub_wrapper<T>::topic_callback, this, std::placeholders::_1)
                        );
                    }

                    const std::string& get_topic_name() const {
                        return m_topic_name;
                    };

                    const std::string& get_msg_type() const {
                        return m_msg_type;
                    }

                private:
                    std::string m_topic_name;
                    std::string m_msg_type;

                    std::shared_ptr<rclcpp::Subscription<T>> m_sub;

                    std::shared_ptr<T> m_data;

                    void topic_callback(const std::shared_ptr<T> msg) {
                        m_data = std::move(msg);
                    }
            };


            std::map<std::string, boost::variant<boost::recursive_wrapper<sub_wrapper<sensor_msgs::msg::Image>>>> m_sub_wrappers;

            std::shared_ptr<rosweb::bridge> m_bridge;

            rclcpp::TimerBase::SharedPtr m_timer;

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

            void bagged_image_to_video(
                const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
                rosweb::server_responses::standard*& res
            );
    };
}

#endif