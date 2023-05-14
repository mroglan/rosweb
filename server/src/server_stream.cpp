#include <iostream>
#include <string>
#include <vector>

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "../include/server_stream.h"
#include "../include/json.hpp"

using json = nlohmann::json_abi_v3_11_2::json;

void rosweb::server_stream::add_msg(const std::string& topic_name,
    const sensor_msgs::msg::Image::SharedPtr msg) {

    std::cout << "Adding image to stream!\n";

    std::cout << msg->height << '\n';

    m_data["topics"][topic_name]["type"] = "sensor_msgs/msg/Image";
    m_data["topics"][topic_name]["ts"] = msg->header.stamp.nanosec;
    std::cout << "stuff0\n";
    // TODO:
    // pass in whether or not the topic is paused
    m_data["topics"][topic_name]["paused"] = false;

    m_data["topics"][topic_name]["data"]["height"] = msg->height;
    m_data["topics"][topic_name]["data"]["width"] = msg->width;

    std::cout << "stuff1\n";
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    std::cout << "stuff2\n";
    std::vector<uchar> data;
    data.assign(cv_ptr->image.data, 
        cv_ptr->image.data + cv_ptr->image.total()*cv_ptr->image.channels());
    m_data["topics"][topic_name]["data"]["data"] = data;
}

void rosweb::server_stream::clear() {
    m_data = {};
    m_data["type"] = "stream";
}

std::string rosweb::server_stream::stringify() const {
    if (!m_data.contains("topics")) return {};

    return m_data.dump();
}