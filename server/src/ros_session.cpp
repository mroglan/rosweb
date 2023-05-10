#include <memory>
#include <chrono>
#include <iostream>
#include <boost/variant.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "../include/ros_session.h"
#include "../include/bridge.h"
#include "../include/client_requests.h"

rosweb::ros_session::ros_session(std::shared_ptr<rosweb::bridge> bridge)
    : Node{"rosweb_ros_session"}, m_bridge{std::move(bridge)} {
    m_timer = create_wall_timer(
        std::chrono::milliseconds{500}, 
        std::bind(&rosweb::ros_session::timer_callback, this)
    );
}

void rosweb::ros_session::timer_callback() {
    std::cout << "Acknowleding request\n";
    handle_new_request();

    for (const auto& w : m_sub_wrappers) {
        std::cout << w.first << '\n';
        if (w.second.which() == 0) {
            std::cout << "From Boost: "
            << boost::get<sub_wrapper<sensor_msgs::msg::Image>>(w.second).get_topic_name() << '\n';
        }
    }
}

void rosweb::ros_session::handle_new_request() {
    auto req_handler = m_bridge->get_client_request_handler();
    if (req_handler->is_acknowledged()) return;

    if (req_handler->get_data()->operation == "create_subscriber") {
        create_subscriber(req_handler);
    } else if (req_handler->get_data()->operation == "destroy_subscriber") {
        destroy_subscriber(req_handler);
    }

    req_handler->acknowledge();
}

void rosweb::ros_session::create_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler) {

    std::cout << "creating subscriber\n";
    auto data = static_cast<const rosweb::client_requests::create_subscriber_request*>(req_handler->get_data());

    if (data->msg_type == "sensor_msgs/msg/Image") {
        m_sub_wrappers.insert({data->topic_name, sub_wrapper<sensor_msgs::msg::Image>{this,data->topic_name}});
    }
}

void rosweb::ros_session::destroy_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler) {
    
    std::cout << "destroying subscriber\n";
    m_sub_wrappers.erase(
        static_cast<const rosweb::client_requests::destroy_subscriber_request*>
        (req_handler->get_data())->topic_name);
}