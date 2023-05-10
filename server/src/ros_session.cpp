#include <memory>
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
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
    req_handler->acknowledge();
}