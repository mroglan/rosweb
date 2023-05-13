#include <iostream>
#include <string>
#include <cstring>

#include "../include/errors.h"


void rosweb::errors::show_critical_error(const std::string& msg) {
    std::cerr << "\033[1;31m" << "ROSWEB CRITICAL ERROR:\n";
    std::cerr << msg << "\033[0m\n";
}

void rosweb::errors::show_noncritical_error(const std::string& msg) {
    std::cerr << "\033[1;31m" << "ROSWEB ERROR:\n";
    std::cerr << msg << "\033[0m\n";
}

rosweb::errors::base_error::base_error(const std::string& msg) 
    : m_msg{msg} {}

rosweb::errors::base_error::~base_error() {}

const char* rosweb::errors::base_error::what() {
    return m_msg.c_str();
}

const std::string& rosweb::errors::base_error::get_msg() const {
    return m_msg;
}

void rosweb::errors::base_error::show() const {
    std::cerr << m_msg << "\033[0m\n";
}

rosweb::errors::message_parse_error::message_parse_error(const std::string& msg) 
    : base_error{msg} {}

rosweb::errors::message_parse_error::~message_parse_error() {}

void rosweb::errors::message_parse_error::show() const {
    std::cerr << "\033[1;31m" << "ROSWEB MESSAGE PARSE ERROR:\n";
    rosweb::errors::base_error::show();
}

rosweb::errors::request_error::request_error(const std::string& msg)
    : base_error{msg} {}

void rosweb::errors::request_error::show() const {
    std::cerr << "\033[1;31m" << "ROSWEB REQUEST ERROR:\n";
    rosweb::errors::base_error::show();
}

rosweb::errors::request_error::~request_error() {}
