#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "../include/bridge.h"
#include "../include/ros_session.h"

int main(int argc, char** argv) {

    auto bridge = std::make_shared<rosweb::bridge>();
    std::thread(&rosweb::bridge::run, bridge.get()).detach();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<rosweb::ros_session>(bridge));
    rclcpp::shutdown();

    return 0;
}