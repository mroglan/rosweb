#include <memory>
#include <boost/beast/core.hpp>

#include "../include/bridge.h"
#include "../include/websocket_session.h"
#include "../include/client_requests.h"
#include "../include/errors.h"
#include "../include/json.hpp"

namespace beast = boost::beast;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

using json = nlohmann::json_abi_v3_11_2::json;

rosweb::bridge::bridge() 
    : m_ioc{1}, m_acceptor{m_ioc, {net::ip::make_address("127.0.0.1"), 8083}},
     m_client_request_handler{new rosweb::client_requests::client_request_handler} {}

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

    try {
        auto j = json::parse(msg);

        if (j["type"] == nullptr) {
            throw rosweb::errors::message_parse_error("No \"type\" field found on incoming message.");
        }

        if (j["type"] == "request") {
            m_client_request_handler->handle_incoming_request(j);
        } else {
            throw rosweb::errors::message_parse_error("Only acceptable incoming message type is "
                "\"request\"");
        }
    } catch (const nlohmann::json_abi_v3_11_2::detail::parse_error& e) {
        rosweb::errors::show_noncritical_error("Error JSON parsing websocket message.");
    } catch (const rosweb::errors::message_parse_error& e) {
        e.show();
    }
    m_session->read();
}