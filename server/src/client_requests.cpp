#include <iostream>
#include <thread>
#include <mutex>

#include "../include/client_requests.h"
#include "../include/errors.h"
#include "../include/supported_ros_types.h"
#include "../include/json.hpp"
#include "../include/websocket_session.h"
#include "../include/server_responses.h"

using json = nlohmann::json_abi_v3_11_2::json;

rosweb::client_requests::client_request_handler::~client_request_handler() {}

void rosweb::client_requests::client_request_handler::handle_incoming_request(json& j,
    std::shared_ptr<rosweb::websocket_session> session) {

    rosweb::server_responses::standard res;

    try {
        if (j["operation"] == nullptr) {
            throw rosweb::errors::message_parse_error("Operation field missing for client request.");
        }
        res.set_operation(j["operation"]);
        if (j["operation"] == "create_subscriber") {
            handle_incoming_subscriber_request(j);
        } else if (j["operation"] == "destroy_subscriber") {
            handle_incoming_destroy_subscriber_request(j);
        } else if (j["operation"] == "change_subscriber") {
            handle_incoming_change_subscriber_request(j);
        } else if (j["operation"] == "bagged_image_to_video") {
            handle_incoming_bagged_image_to_video_request(j);
        } else if (j["operation"] == "toggle_pause_subscriber") {
            handle_incoming_toggle_pause_subscriber_request(j);
        } else {
            throw rosweb::errors::message_parse_error("No valid operation field value provided for client request.");
        }
    } catch (const rosweb::errors::base_error& e) {
        res.set_status(400);
        res.set_msg(e.get_msg());
        session->queue_messages({res.stringify()});
        e.show();
    }
}

void rosweb::client_requests::client_request_handler::acknowledge() {
    std::unique_lock<std::mutex> lock{m_mutex};
    m_acknowledged = true;
    lock.unlock();
    m_cv.notify_one();
}

bool rosweb::client_requests::client_request_handler::is_acknowledged() const {
    std::lock_guard<std::mutex> guard{m_mutex};
    return m_acknowledged;
}

const rosweb::client_requests::client_request* 
    rosweb::client_requests::client_request_handler::get_data() const {
    return m_data.get();
}

void rosweb::client_requests::client_request_handler::handle_incoming_subscriber_request(json& j) {
    if (j["data"]["topic_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.topic_name.");
    }
    if (j["data"]["msg_type"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.msg_type.");
    }
    rosweb::supported_ros_types::verify_is_supported_msg(j["data"]["msg_type"]);

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [&ack = m_acknowledged]{return ack;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::create_subscriber_request;
    data->operation = j["operation"];
    data->msg_type = j["data"]["msg_type"];
    data->topic_name = j["data"]["topic_name"];

    m_data = std::unique_ptr<rosweb::client_requests::create_subscriber_request>(data);
}

void rosweb::client_requests::client_request_handler::handle_incoming_destroy_subscriber_request(json& j) {
    if (j["data"]["topic_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.topic_name.");
    }
    if (j["data"]["msg_type"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.msg_type.");
    }

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [&ack = m_acknowledged]{return ack;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::destroy_subscriber_request;
    data->operation = j["operation"];
    data->msg_type = j["data"]["msg_type"];
    data->topic_name = j["data"]["topic_name"];

    m_data = std::unique_ptr<rosweb::client_requests::destroy_subscriber_request>(data);
}

void rosweb::client_requests::client_request_handler::handle_incoming_change_subscriber_request(json& j) {
    if (j["data"]["prev_topic_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.prev_topic_name.");
    }
    if (j["data"]["new_topic_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.new_topic_name.");
    }
    if (j["data"]["msg_type"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.msg_type.");
    }

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [&ack = m_acknowledged]{return ack;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::change_subscriber_request;
    data->operation = j["operation"];
    data->msg_type = j["data"]["msg_type"];
    data->new_topic_name = j["data"]["new_topic_name"];
    data->prev_topic_name = j["data"]["prev_topic_name"];

    m_data = std::unique_ptr<rosweb::client_requests::change_subscriber_request>(data);
}

void rosweb::client_requests::client_request_handler::handle_incoming_toggle_pause_subscriber_request(json& j) {
    if (j["data"]["topic_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.topic_name.");
    }
    if (j["data"]["msg_type"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.msg_type.");
    }

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [&ack = m_acknowledged]{return ack;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::toggle_pause_subscriber_request;
    data->operation = j["operation"];
    data->msg_type = j["data"]["msg_type"];
    data->topic_name = j["data"]["topic_name"];

    m_data = std::unique_ptr<rosweb::client_requests::toggle_pause_subscriber_request>(data);
}

void rosweb::client_requests::client_request_handler::handle_incoming_bagged_image_to_video_request(json& j) {
    if (j["data"]["output_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.output_name.");
    }
    if (j["data"]["bag_path"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.bag_path.");
    }
    if (j["data"]["topic_name"] == nullptr) {
        throw rosweb::errors::request_error("Missing required field data.topic_name.");
    }

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [&ack = m_acknowledged]{return ack;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::bagged_image_to_video_request;
    data->operation = j["operation"];
    data->output_name = j["data"]["output_name"];
    data->bag_path = j["data"]["bag_path"];
    data->topic_name = j["data"]["topic_name"];
    data->create_html = j["data"]["create_html"];

    m_data = std::unique_ptr<rosweb::client_requests::bagged_image_to_video_request>(data);
}

rosweb::client_requests::client_request::~client_request() {}

rosweb::client_requests::create_subscriber_request::~create_subscriber_request() {}

rosweb::client_requests::destroy_subscriber_request::~destroy_subscriber_request() {}

rosweb::client_requests::bagged_image_to_video_request::~bagged_image_to_video_request() {}

rosweb::client_requests::change_subscriber_request::~change_subscriber_request() {}

rosweb::client_requests::toggle_pause_subscriber_request::~toggle_pause_subscriber_request() {}