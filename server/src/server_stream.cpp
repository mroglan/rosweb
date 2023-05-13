#include <iostream>
#include <string>

#include "sensor_msgs/msg/image.hpp"
#include "../include/server_stream.h"

void rosweb::server_stream::add_msg(const std::string& topic_name,
    const sensor_msgs::msg::Image* msg) {
    auto iter = m_data.find(topic_name);
    if (iter != m_data.end() && 
        iter->second["ts"] == msg->header.stamp.nanosec) return;
    
    std::cout << "Adding image to stream!\n";
}

std::string rosweb::server_stream::stringify() const {
    return "";
}