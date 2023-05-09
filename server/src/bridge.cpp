#include <memory>
#include <boost/beast/core.hpp>

#include "../include/bridge.h"
#include "../include/websocket_session.h"
#include "../include/errors.h"

namespace beast = boost::beast;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

rosweb::bridge::bridge() 
    : m_ioc{1}, m_acceptor{m_ioc, {net::ip::make_address("127.0.0.1"), 8083}} {}

void rosweb::bridge::run() {
    accept();
    m_ioc.run();
}

void rosweb::bridge::accept() {
    m_acceptor.async_accept(m_ioc, 
        beast::bind_front_handler(&rosweb::bridge::on_accept, shared_from_this()));
}

void rosweb::bridge::on_accept(beast::error_code ec, tcp::socket socket) {
    if (ec) {
        rosweb::errors::show_noncritical_error("Error accepting socket connection. Will retry.");
    } else if (!m_session.get()) {
        std::cout << "Accepted socket connection.\n";
        m_session = std::make_shared<rosweb::websocket_session>(shared_from_this(), std::move(socket));
        m_session->run();
    } else if (m_session->is_closed()) {
        std::cout << "Reestablished socket connection.\n";
        m_session = std::make_shared<rosweb::websocket_session>(shared_from_this(), std::move(socket));
        m_session->run();
    } else {
        rosweb::errors::show_noncritical_error("Already connected to an active session. " 
            "Multiple sessions currently not supported.");
    }
    accept();
}

void rosweb::bridge::handle_incoming_ws_msg(const std::string& msg) {
    std::cout << "Message: " << msg << '\n';
    m_session->read();
}