#include <memory>
#include <thread>
#include <mutex>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include "../include/websocket_session.h"
#include "../include/errors.h"

namespace beast = boost::beast;
using tcp = boost::asio::ip::tcp;

rosweb::websocket_session::websocket_session(std::shared_ptr<rosweb::bridge> bridge, 
    tcp::socket&& socket)
    : m_bridge{std::move(bridge)}, m_ws{std::move(socket)} {}

void rosweb::websocket_session::run() {
    std::lock_guard<std::mutex> guard{m_mutex};

    m_ws.async_accept(beast::bind_front_handler(&rosweb::websocket_session::on_accept, shared_from_this()));
}

void rosweb::websocket_session::on_accept(beast::error_code ec) {
    if (ec) {
        rosweb::errors::show_critical_error("Error accepting websocket session.");
        return;
    }
    std::cout << "Accepted websocket connection.\n";
}