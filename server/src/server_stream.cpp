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

    auto iter = m_timestamps.find(topic_name);
    if (iter != m_timestamps.end() 
        && iter->second == msg->header.stamp.nanosec) return;
    
    m_timestamps[topic_name] = msg->header.stamp.nanosec;

    m_data["topics"][topic_name]["type"] = "sensor_msgs/msg/Image";
    m_data["topics"][topic_name]["ts"] = msg->header.stamp.nanosec;

    m_data["topics"][topic_name]["data"]["height"] = msg->height;
    m_data["topics"][topic_name]["data"]["width"] = msg->width;

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8");
    std::vector<uchar> data;
    data.assign(cv_ptr->image.data, 
        cv_ptr->image.data + cv_ptr->image.total()*cv_ptr->image.channels());
    m_data["topics"][topic_name]["data"]["data"] = data;
}

void rosweb::server_stream::add_msg(const std::string& topic_name,
    const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    
    auto iter = m_timestamps.find(topic_name);
    if (iter != m_timestamps.end() 
        && iter->second == msg->header.stamp.nanosec) return;
    
    m_timestamps[topic_name] = msg->header.stamp.nanosec;

    m_data["topics"][topic_name]["type"] = "sensor_msgs/msg/NavSatFix";
    m_data["topics"][topic_name]["ts"] = msg->header.stamp.nanosec;

    m_data["topics"][topic_name]["data"]["latitude"] = msg->latitude;
    m_data["topics"][topic_name]["data"]["longitude"] = msg->longitude;
    m_data["topics"][topic_name]["data"]["altitude"] = msg->altitude;
}

void rosweb::server_stream::add_msg(const std::string& topic_name,
    const nav_msgs::msg::Odometry::SharedPtr msg) {

    auto iter = m_timestamps.find(topic_name);
    if (iter != m_timestamps.end() 
        && iter->second == msg->header.stamp.nanosec) return;
    
    m_timestamps[topic_name] = msg->header.stamp.nanosec;

    m_data["topics"][topic_name]["type"] = "nav_msgs/msg/Odometry";
    m_data["topics"][topic_name]["ts"] = msg->header.stamp.nanosec;

    m_data["topics"][topic_name]["data"]["pose"]["position"]["x"] = msg->pose.pose.position.x;
    m_data["topics"][topic_name]["data"]["pose"]["position"]["y"] = msg->pose.pose.position.y;
    m_data["topics"][topic_name]["data"]["pose"]["position"]["z"] = msg->pose.pose.position.z;

    m_data["topics"][topic_name]["data"]["pose"]["orientation"]["x"] = msg->pose.pose.orientation.x;
    m_data["topics"][topic_name]["data"]["pose"]["orientation"]["y"] = msg->pose.pose.orientation.y;
    m_data["topics"][topic_name]["data"]["pose"]["orientation"]["z"] = msg->pose.pose.orientation.z;
    m_data["topics"][topic_name]["data"]["pose"]["orientation"]["w"] = msg->pose.pose.orientation.w;
}

void rosweb::server_stream::clear() {
    m_data = {};
    m_data["type"] = "stream";
}

void rosweb::server_stream::reset_timestamps() {
    m_timestamps.clear();
    std::cout << "reset stream" << std::endl;
}

std::string rosweb::server_stream::stringify() const {
    if (!m_data.contains("topics")) return {};

    return m_data.dump();
}