#include <string>

#include "../include/json.hpp"
#include "../include/server_responses.h"

using json = nlohmann::json_abi_v3_11_2::json;

void rosweb::server_responses::standard::set_operation(const std::string& operation) {
    m_operation = operation;
}

void rosweb::server_responses::standard::set_msg(const std::string& msg) {
    m_msg = msg;
}

void rosweb::server_responses::standard::set_status(int status) {
    m_status = status;
}

json rosweb::server_responses::standard::json() {
    nlohmann::json_abi_v3_11_2::json j;

    j["type"] = "response";
    j["operation"] = m_operation;
    j["data"]["msg"] = m_msg;
    j["data"]["status"] = m_status;

    return j;
}

std::string rosweb::server_responses::standard::stringify() {
    return json().dump();
}

bool rosweb::server_responses::standard::operator!() const {
    return m_operation.empty();
}