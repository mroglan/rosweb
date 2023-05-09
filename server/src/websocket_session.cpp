#include <memory>
#include <thread>
#include <mutex>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include "../include/websocket_session.h"
#include "../include/bridge.h"
#include "../include/errors.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
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
    read();
}

void rosweb::websocket_session::read() {
    std::lock_guard<std::mutex> guard{m_mutex};
    m_ws.async_read(m_buffer, 
        beast::bind_front_handler(&rosweb::websocket_session::on_read, shared_from_this()));
}

void rosweb::websocket_session::on_read(beast::error_code ec, std::size_t bytes_transferred) {
    std::unique_lock<std::mutex> lock{m_mutex};

    if (ec == websocket::error::closed) {
        rosweb::errors::show_critical_error("Websocket connection closed.");
        m_buffer.clear();
        return;
    }
    if (ec) {
        rosweb::errors::show_noncritical_error("Error reading websocket message.");
        m_buffer.clear();
        lock.unlock();
        read();
        return;
    }

    auto msg = beast::buffers_to_string(m_buffer.cdata());

    m_buffer.clear();
    lock.unlock();

    m_bridge->handle_incoming_ws_msg(msg);
}