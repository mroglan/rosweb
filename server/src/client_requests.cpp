#include <iostream>
#include <thread>
#include <mutex>

#include "../include/client_requests.h"
#include "../include/errors.h"
#include "../include/json.hpp"

using json = nlohmann::json_abi_v3_11_2::json;

void rosweb::client_requests::client_request_handler::handle_incoming_request(json& j) {

    try {
        if (j["operation"] == nullptr) {
            throw rosweb::errors::message_parse_error("Operation field missing for client request.");
        }
        if (j["operation"] == "create_subscriber") {
            handle_incoming_subscriber_request(j);
        } else {
            throw rosweb::errors::message_parse_error("No valid operation field value provided for client request.");
        }
    } catch (const rosweb::errors::message_parse_error& e) {
        e.show();
    }
}

void rosweb::client_requests::client_request_handler::handle_incoming_subscriber_request(json& j) {
    if (j["data"]["topic_name"] == nullptr) {
        throw rosweb::errors::message_parse_error("Missing required field data.topic_name.");
    }
    if (j["data"]["msg_type"] == nullptr) {
        throw rosweb::errors::message_parse_error("Missing required field data.msg_type.");
    }

    std::unique_lock<std::mutex> lock{m_mutex};
    m_cv.wait(lock, [this]{return m_acknowledged;});

    m_acknowledged = false;

    auto data = new rosweb::client_requests::create_subscriber_request;
    data->operation = j["operation"];
    data->msg_type = j["data"]["msg_type"];
    data->topic_name = j["data"]["topic_name"];

    m_data = std::unique_ptr<rosweb::client_requests::create_subscriber_request>(data);

    std::cout << "Added Data!\n";
}

rosweb::client_requests::client_request::~client_request() {}