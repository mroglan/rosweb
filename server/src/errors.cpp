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

rosweb::errors::message_parse_error::message_parse_error(const std::string& msg) 
    : m_msg{msg} {}

const char* rosweb::errors::message_parse_error::what() {
    return m_msg.c_str();
}

void rosweb::errors::message_parse_error::show() const {
    std::cerr << "\033[1;31m" << "ROSWEB MESSAGE PARSE ERROR:\n";
    std::cerr << m_msg << "\033[0m\n";
}
