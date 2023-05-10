#include <memory>
#include <chrono>
#include <iostream>

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

    auto req_handler = m_bridge->get_client_request_handler();
    if (req_handler->is_acknowledged()) return;

    if (req_handler->get_data()->operation == "create_subscriber") {
        create_subscriber(req_handler);
    }

    req_handler->acknowledge();
}

void rosweb::ros_session::create_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler) {
    std::cout << "creating subscriber\n";
    auto data = static_cast<const rosweb::client_requests::create_subscriber_request*>(req_handler->get_data());

    void* wrapper;
    if (data->msg_type == "sensor_msgs/msg/Image") {
        wrapper = new sub_wrapper<sensor_msgs::msg::Image>{this, data->topic_name};
    }
    m_sub_wrappers.insert({data->topic_name, wrapper});
}