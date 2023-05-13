#include <iostream>
#include <string>

#include "../include/json.hpp"
#include "../include/server_responses.h"

void rosweb::server_responses::standard::set_operation(const std::string& operation) {
    m_operation = operation;
}

void rosweb::server_responses::standard::set_msg(const std::string& msg) {
    m_msg = msg;
}

void rosweb::server_responses::standard::set_status(int status) {
    m_status = status;
}

nlohmann::json_abi_v3_11_2::json rosweb::server_responses::standard::json() const {
    nlohmann::json_abi_v3_11_2::json j;

    j["type"] = "response";
    j["operation"] = m_operation;
    j["data"]["msg"] = m_msg;
    j["data"]["status"] = m_status;

    return j;
}

std::string rosweb::server_responses::standard::stringify() const {
    return json().dump();
}

bool rosweb::server_responses::standard::operator!() const {
    return m_operation.empty();
}

rosweb::server_responses::standard::~standard() {}

void rosweb::server_responses::create_or_destroy_sub::set_topic_name(const std::string& topic_name)  {
    m_topic_name = topic_name;
}

void rosweb::server_responses::create_or_destroy_sub::set_msg_type(const std::string& msg_type) {
    m_msg_type = msg_type;
}

std::string rosweb::server_responses::create_or_destroy_sub::stringify() const {
    nlohmann::json_abi_v3_11_2::json j = json();

    j["data"]["topic_name"] = m_topic_name;
    j["data"]["msg_type"] = m_msg_type;

    return j.dump();
}

rosweb::server_responses::create_or_destroy_sub::~create_or_destroy_sub() {}

void rosweb::server_responses::change_sub::set_new_topic_name(const std::string& topic_name) {
    m_new_topic_name = topic_name;
}

void rosweb::server_responses::change_sub::set_prev_topic_name(const std::string& topic_name) {
    m_prev_topic_name = topic_name;
}

void rosweb::server_responses::change_sub::set_msg_type(const std::string& msg_type) {
    m_msg_type = msg_type;
}

std::string rosweb::server_responses::change_sub::stringify() const {
    nlohmann::json_abi_v3_11_2::json j = json();

    j["data"]["new_topic_name"] = m_new_topic_name;
    j["data"]["prev_topic_name"] = m_prev_topic_name;
    j["data"]["msg_type"] = m_msg_type;

    return j.dump();
}

rosweb::server_responses::change_sub::~change_sub() {}