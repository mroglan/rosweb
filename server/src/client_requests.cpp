#include <iostream>
#include <thread>
#include <mutex>

#include "../include/client_requests.h"
#include "../include/errors.h"
#include "../include/json.hpp"

using json = nlohmann::json_abi_v3_11_2::json;

rosweb::client_requests::client_request_handler::~client_request_handler() {}

void rosweb::client_requests::client_request_handler::handle_incoming_request(json& j) {

    try {
        if (j["operation"] == nullptr) {
            throw rosweb::errors::message_parse_error("Operation field missing for client request.");
        }
        if (j["operation"] == "create_subscriber") {
            handle_incoming_subscriber_request(j);
        } else if (j["operation"] == "destroy_subscriber") {
            handle_incoming_destroy_subscriber_request(j);
        } else {
            throw rosweb::errors::message_parse_error("No valid operation field value provided for client request.");
        }
    } catch (const rosweb::errors::message_parse_error& e) {
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
        throw rosweb::errors::message_parse_error("Missing required field data.topic_name.");
    }
    if (j["data"]["msg_type"] == nullptr) {
        throw rosweb::errors::message_parse_error("Missing required field data.msg_type.");
    }

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
        throw rosweb::errors::message_parse_error("Missing required field data.topic_name.");
    }

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [&ack = m_acknowledged]{return ack;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::destroy_subscriber_request;
    data->operation = j["operation"];
    data->topic_name = j["data"]["topic_name"];

    m_data = std::unique_ptr<rosweb::client_requests::destroy_subscriber_request>(data);
}

rosweb::client_requests::client_request::~client_request() {}

rosweb::client_requests::create_subscriber_request::~create_subscriber_request() {}

rosweb::client_requests::destroy_subscriber_request::~destroy_subscriber_request() {}