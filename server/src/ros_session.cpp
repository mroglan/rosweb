#include <memory>
#include <chrono>
#include <iostream>
#include <fstream>
#include <boost/variant.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rosbag2_cpp/reader.hpp"

#include "../include/ros_session.h"
#include "../include/bridge.h"
#include "../include/client_requests.h"
#include "../include/errors.h"
#include "../include/server_responses.h"
#include "../include/json.hpp"

using json = nlohmann::json_abi_v3_11_2::json;

rosweb::ros_session::ros_session(std::shared_ptr<rosweb::bridge> bridge)
    : Node{"rosweb_ros_session"}, m_bridge{std::move(bridge)} {
    m_timer = create_wall_timer(
        std::chrono::milliseconds{100}, 
        std::bind(&rosweb::ros_session::timer_callback, this)
    );
}

void rosweb::ros_session::timer_callback() {
    rosweb::server_responses::standard res;
    handle_new_request(res);
    std::cout << "Response: " << res.stringify() << '\n';

    for (const auto& w : m_sub_wrappers) {
        std::cout << w.first << '\n';
        if (w.second.which() == 0) {
            std::cout << "From Boost: "
            << boost::get<sub_wrapper<sensor_msgs::msg::Image>>(w.second).get_topic_name() << '\n';
        }
    }
}

void rosweb::ros_session::handle_new_request(rosweb::server_responses::standard& res) {
    auto req_handler = m_bridge->get_client_request_handler();
    if (req_handler->is_acknowledged()) return;

    try {
        if (req_handler->get_data()->operation == "create_subscriber") {
            create_subscriber(req_handler);
        } else if (req_handler->get_data()->operation == "destroy_subscriber") {
            destroy_subscriber(req_handler);
        } else if (req_handler->get_data()->operation == "bagged_image_to_video") {
            bagged_image_to_video(req_handler);
        }
        // TEMPORARY
        res.set_operation(req_handler->get_data()->operation);
        res.set_status(200);
        res.set_msg("Operation successful");
    } catch (const rosweb::errors::request_error& e) {
        // TEMPORARY
        res.set_operation(req_handler->get_data()->operation);
        res.set_status(500);
        res.set_msg("Operation failed");
        e.show();
    }

    req_handler->acknowledge();
}

void rosweb::ros_session::create_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler) {

    auto data = static_cast<const rosweb::client_requests::create_subscriber_request*>(req_handler->get_data());

    if (m_sub_wrappers.find(data->topic_name) != m_sub_wrappers.end()) {
        throw rosweb::errors::request_error("Subscription to " + data->topic_name + " already exists.");
    }

    std::cout << "Creating subscriber to " << data->topic_name << '\n';

    if (data->msg_type == "sensor_msgs/msg/Image") {
        m_sub_wrappers.insert({data->topic_name, sub_wrapper<sensor_msgs::msg::Image>{this,data->topic_name}});
    }
}

void rosweb::ros_session::destroy_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler) {

    std::string topic_name = static_cast<const rosweb::client_requests::destroy_subscriber_request*>
        (req_handler->get_data())->topic_name;
    
    if (m_sub_wrappers.find(topic_name) == m_sub_wrappers.end()) {
        throw rosweb::errors::request_error("No subscription to " + topic_name + " to destroy.");
    }
    
    std::cout << "Destroying subscriber to " << topic_name << '\n';
    m_sub_wrappers.erase(topic_name);
}

void rosweb::ros_session::bagged_image_to_video(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler) {

    auto data = static_cast<const rosweb::client_requests::bagged_image_to_video_request*>
        (req_handler->get_data());
    
    if (!getenv("HOME")) {
        rosweb::errors::show_noncritical_error("No HOME directory found, cannot convert" 
            "ROS bag to video.");
        return;
    }

    std::string home_dir{getenv("HOME")};
    
    try {
        rclcpp::Serialization<sensor_msgs::msg::Image> serialization;

        rosbag2_cpp::Reader reader;
        reader.open(home_dir + '/' + data->bag_path);

        sensor_msgs::msg::Image::SharedPtr ros_msg = std::make_shared<sensor_msgs::msg::Image>();

        std::vector<cv_bridge::CvImagePtr> cv_ptrs;
        
        while (reader.has_next()) {
            auto msg = reader.read_next();

            if (msg->topic_name != data->topic_name) continue;

            rclcpp::SerializedMessage extracted_serialized_msg(*msg->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg, ros_msg.get());

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_msg, "rgb8");
            
            cv_ptrs.push_back(cv_ptr);
        }

        reader.close();

        cv::VideoWriter out(home_dir + "/Downloads/" + data->output_name + ".mp4", 
            cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 
            10.0, cv_ptrs[0]->image.size());
        
        for (const cv_bridge::CvImagePtr& cv_ptr : cv_ptrs) {
            out.write(cv_ptr->image);
        }

        out.release();

        std::cout << "Created video from ROS bag!\n";

    } catch (const std::exception& e) {
        rosweb::errors::show_noncritical_error("Failed to convert ROS bag to video.");
        std::cout << e.what() << '\n';
        return;
    }

    try {
        if (data->create_html) {
            std::string file_path = home_dir + "/Downloads/" + data->output_name + + ".html";

            std::ofstream ofs;
            ofs.open(file_path, std::ofstream::out | std::ofstream::trunc);
            ofs << "<html>\n";
            ofs << "<head>\n";
            ofs << "<title>" << data->output_name << "</title>\n";
            ofs << "</head>\n";
            ofs << "<body>\n";
            ofs << "<div style=\"text-align:center;\">\n";
            ofs << "<h1>" << data->output_name << "</h1>\n";
            ofs << "<video src=\"" << home_dir << "/Downloads/" << data->output_name << ".mp4\" controls />\n";
            ofs << "</div>\n";
            ofs << "</body>\n";
            ofs << "</html>\n";

            ofs.close();

            std::cout << "Created HTML to view video!\n";

            std::string command = "open " + file_path;
            system(command.c_str());
        }
    } catch (const std::exception& e) {
        rosweb::errors::show_noncritical_error("Failed to create HTML to view video.");
        std::cout << e.what() << '\n';
        return;
    }
}