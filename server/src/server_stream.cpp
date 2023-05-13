#include <iostream>
#include <string>
#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "../include/server_stream.h"
#include "../include/json.hpp"

using json = nlohmann::json_abi_v3_11_2::json;

void rosweb::server_stream::add_msg(const std::string& topic_name,
    const sensor_msgs::msg::Image& msg) {
    if (!msg.header.stamp.nanosec) return;

    auto iter = m_data.find(topic_name);
    if (iter != m_data.end() && 
        iter->second["ts"] == msg.header.stamp.nanosec) return;
    
    std::cout << "Adding image to stream!\n";

    // std::cout << msg.get() << '\n';
    std::cout << msg.height << '\n';
    
    json j;

    j["type"] = "sensor_msgs/msg/Image";
    j["ts"] = msg.header.stamp.nanosec;
    std::cout << "stuff0\n";
    // TODO:
    // pass in whether or not the topic is paused
    j["paused"] = false;

    j["data"]["height"] = msg.height;
    j["data"]["width"] = msg.width;
    
    std::cout << "stuff1\n";
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    std::cout << "stuff2\n";
    std::vector<uchar> data;
    data.assign(cv_ptr->image.data, 
        cv_ptr->image.data + cv_ptr->image.total()*cv_ptr->image.channels());
    j["data"]["data"] = data;

    m_data[topic_name] = j;
}

std::string rosweb::server_stream::stringify() const {
    return "";
}