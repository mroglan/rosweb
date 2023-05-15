#include <memory>
#include <string>
#include <vector>
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
#include "../include/server_stream.h"
#include "../include/json.hpp"

using json = nlohmann::json_abi_v3_11_2::json;

rosweb::ros_session::ros_session(std::shared_ptr<rosweb::bridge> bridge)
    : Node{"rosweb_ros_session"}, m_bridge{std::move(bridge)}, 
    m_stream{new rosweb::server_stream} {
    m_timer = create_wall_timer(
        std::chrono::milliseconds{100}, 
        std::bind(&rosweb::ros_session::timer_callback, this)
    );
}

void rosweb::ros_session::timer_callback() {
    std::vector<std::string> msgs;

    auto res = new rosweb::server_responses::standard;
    handle_new_request(res);

    if (!!*res) {
        msgs.push_back(res->stringify());
    }
    delete res;

    auto start = std::chrono::high_resolution_clock::now();
    m_stream->clear();
    for (const auto& w : m_sub_wrapper.image_data) {
        if (!w.second) continue;
        if (m_sub_wrapper.paused.find(w.first) != m_sub_wrapper.paused.end()) continue;
        m_stream->add_msg(w.first, w.second);
    }

    std::string s = m_stream->stringify();
    if (!s.empty()) {
        msgs.push_back(s);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Duration: " << duration.count() << '\n';

    m_bridge->handle_outgoing_ws_msgs(msgs);
}

void rosweb::ros_session::handle_new_request(rosweb::server_responses::standard*& res) {
    auto req_handler = m_bridge->get_client_request_handler();
    if (req_handler->is_acknowledged()) return;

    if (req_handler->get_data()->operation == "create_subscriber") {
        create_subscriber(req_handler, res);
    } else if (req_handler->get_data()->operation == "destroy_subscriber") {
        destroy_subscriber(req_handler, res);
    } else if (req_handler->get_data()->operation == "change_subscriber") {
        change_subscriber(req_handler, res);
    } else if (req_handler->get_data()->operation == "toggle_pause_subscriber") {
        toggle_pause_subscriber(req_handler, res);
    } else if (req_handler->get_data()->operation == "bagged_image_to_video") {
        bagged_image_to_video(req_handler, res);
    }
    res->set_operation(req_handler->get_data()->operation);

    req_handler->acknowledge();
}

void rosweb::ros_session::create_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
    rosweb::server_responses::standard*& res) {

    delete res;
    res = new rosweb::server_responses::create_or_destroy_sub;

    auto data = static_cast<const rosweb::client_requests::create_subscriber_request*>(req_handler->get_data());

    static_cast<rosweb::server_responses::create_or_destroy_sub*>(res)->set_topic_name(data->topic_name);
    static_cast<rosweb::server_responses::create_or_destroy_sub*>(res)->set_msg_type(data->msg_type);

    if (m_sub_wrapper.types.find(data->topic_name) != m_sub_wrapper.types.end()) {
        res->set_status(400);
        res->set_msg("Subscription already exists.");
        rosweb::errors::request_error("Subscription to " + data->topic_name + " already exists.").show();
        return;
    }

    std::cout << "Creating subscriber to " << data->topic_name << '\n';
    create_sub_helper(data->topic_name, data->msg_type);

    res->set_status(200);
    res->set_msg("Successfully created subscription.");
}

void rosweb::ros_session::destroy_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
    rosweb::server_responses::standard*& res) {

    delete res;
    res = new rosweb::server_responses::create_or_destroy_sub;

    auto data = static_cast<const rosweb::client_requests::destroy_subscriber_request*>(req_handler->get_data());

    static_cast<rosweb::server_responses::create_or_destroy_sub*>(res)->set_topic_name(data->topic_name);
    static_cast<rosweb::server_responses::create_or_destroy_sub*>(res)->set_msg_type(data->msg_type);
    
    if (m_sub_wrapper.types.find(data->topic_name) == m_sub_wrapper.types.end()) {
        res->set_status(400);
        res->set_msg("No subscription to destroy.");
        rosweb::errors::request_error("No subscription to " + data->topic_name + " to destroy.").show();
        return;
    } 
    
    std::cout << "Destroying subscriber to " << data->topic_name << '\n';
    destroy_sub_helper(data->topic_name, data->msg_type);

    res->set_status(200);
    res->set_msg("Successfully destroyed subscription.");
}

void rosweb::ros_session::change_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
    rosweb::server_responses::standard*& res) {
    
    delete res;
    res = new rosweb::server_responses::change_sub;

    auto data = static_cast<const rosweb::client_requests::change_subscriber_request*>(req_handler->get_data());

    auto r = static_cast<rosweb::server_responses::change_sub*>(res);
    r->set_new_topic_name(data->new_topic_name);
    r->set_prev_topic_name(data->prev_topic_name);
    r->set_msg_type(data->msg_type);

    if (m_sub_wrapper.types.find(data->new_topic_name) != m_sub_wrapper.types.end()) {
        r->set_status(400);
        r->set_msg("Subscription already exists.");
        rosweb::errors::request_error("Subscription to " + data->new_topic_name + " already exists.").show();
        return;
    }

    if (m_sub_wrapper.types.find(data->prev_topic_name) == m_sub_wrapper.types.end()) {
        r->set_status(400);
        r->set_msg("No subscription to destroy.");
        rosweb::errors::request_error("No subscription to " + data->prev_topic_name + " to destroy.").show();
        return;
    }

    if (m_sub_wrapper.types[data->prev_topic_name] != data->msg_type) {
        r->set_status(400);
        r->set_msg("Msg type does not match current msg type.");
        rosweb::errors::request_error("Previous msg type of " + m_sub_wrapper.types[data->prev_topic_name]
            + " and requested msg type of " + data->msg_type).show();
        return;
    }

    std::cout << "Changing subscription from " << data->prev_topic_name 
        << " to " << data->new_topic_name << '\n';

    destroy_sub_helper(data->prev_topic_name, data->msg_type);
    create_sub_helper(data->new_topic_name, data->msg_type);

    r->set_status(200);
    r->set_msg("Successfully changed subscription.");
}

void rosweb::ros_session::toggle_pause_subscriber(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
    rosweb::server_responses::standard*& res) {
    
    delete res;
    res = new rosweb::server_responses::create_or_destroy_sub;

    auto data = static_cast<const rosweb::client_requests::toggle_pause_subscriber_request*>
        (req_handler->get_data());
    
    auto r = static_cast<rosweb::server_responses::create_or_destroy_sub*>(res);
    r->set_topic_name(data->topic_name);
    r->set_msg_type(data->msg_type);

    if (m_sub_wrapper.types.find(data->topic_name) == m_sub_wrapper.types.end()) {
        r->set_status(400);
        r->set_msg("No subscription to toggle pause.");
        rosweb::errors::request_error("No subscription to " + data->topic_name + " to toggle pause.").show();
        return;
    }

    if (m_sub_wrapper.paused.find(data->topic_name) == m_sub_wrapper.paused.end()) {
        std::cout << "Pausing subscription to " << data->topic_name << '\n';
        m_sub_wrapper.paused.insert(data->topic_name);
        r->set_msg("Paused subscription.");
    } else {
        std::cout << "Playing subscription to " << data->topic_name << '\n';
        m_sub_wrapper.paused.erase(data->topic_name);
        r->set_msg("Playing subscription.");
    }
    r->set_status(200);
}

void rosweb::ros_session::create_sub_helper(const std::string& topic_name, const std::string& msg_type) {
    m_sub_wrapper.types.insert({topic_name, msg_type});
    if (msg_type == "sensor_msgs/msg/Image") {
        m_sub_wrapper.image_data.insert({topic_name, std::shared_ptr<sensor_msgs::msg::Image>()});
        m_sub_wrapper.image_subs.insert({topic_name, create_subscription<sensor_msgs::msg::Image>(
            topic_name, 10, [&wrapper = m_sub_wrapper, topic_name](sensor_msgs::msg::Image::SharedPtr msg){
                wrapper.image_data[topic_name] = std::move(msg);
            }
        )});
    }
}

void rosweb::ros_session::destroy_sub_helper(const std::string& topic_name, const std::string& msg_type) {
    m_sub_wrapper.types.erase(topic_name);
    if (msg_type == "sensor_msgs/msg/Image") {
        m_sub_wrapper.image_subs.erase(topic_name);
        m_sub_wrapper.image_data.erase(topic_name);
    }
}

void rosweb::ros_session::bagged_image_to_video(
    const std::shared_ptr<rosweb::client_requests::client_request_handler>& req_handler,
    rosweb::server_responses::standard*& res) {
    
    auto data = static_cast<const rosweb::client_requests::bagged_image_to_video_request*>
        (req_handler->get_data());
    
    if (!getenv("HOME")) {
        res->set_status(500);
        res->set_msg("Unable to find HOME directory.");
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

        res->set_status(200);
    } catch (const std::exception& e) {
        res->set_status(500);
        res->set_msg("Failed to create video. See full error in server terminal.");
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

            res->set_msg("Video and HTML file created! If the HTML was not opened in a browser "
                "automatically, you can find it in your Downloads.");
        } else {
            res->set_msg("Video created!");
        }
    } catch (const std::exception& e) {
        res->set_msg("Video created, but failed to create HTML file. See full error in "
        "server terminal.");
        rosweb::errors::show_noncritical_error("Failed to create HTML to view video.");
        std::cout << e.what() << '\n';
        return;
    }
}