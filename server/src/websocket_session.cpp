#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/bind/bind.hpp>

#include "../include/websocket_session.h"
#include "../include/bridge.h"
#include "../include/errors.h"

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

rosweb::websocket_session::websocket_session(std::shared_ptr<rosweb::bridge> bridge, 
    tcp::socket&& socket, net::io_context& ioc)
    : m_bridge{std::move(bridge)}, m_ws{std::move(socket)}, 
    m_write_timer{ioc, boost::posix_time::milliseconds(100)} {}

rosweb::websocket_session::~websocket_session() {
    std::cout << "God bless the weak_ptr. Memory leaks no more!\n";
}

void rosweb::websocket_session::run() {
    std::lock_guard<std::mutex> guard{m_mutex};

    m_write_timer.async_wait(boost::bind(&websocket_session::write_next_pending, shared_from_this()));
    m_ws.async_accept(beast::bind_front_handler(&rosweb::websocket_session::on_accept, shared_from_this()));
}

bool rosweb::websocket_session::is_closed() {
    std::lock_guard<std::mutex> guard{m_mutex};

    return m_session_closed;
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

    // 125 = operation cancelled
    if (ec == websocket::error::closed || ec.value() == 125) {
        rosweb::errors::show_noncritical_error("Websocket connection closed. Will attempt to reconnect.");
        m_session_closed = true;
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

    auto bridge = m_bridge.lock();

    bridge->handle_incoming_ws_msg(msg);
}

void rosweb::websocket_session::queue_messages(const std::vector<std::string>& msgs) {
    if (m_session_closed) return;
    std::lock_guard<std::mutex> lock{m_mutex};

    for (const auto& msg : msgs) {
        m_pending_writes.push(msg);
    }
}

void rosweb::websocket_session::write_next_pending() {
    if (m_session_closed) return;

    std::lock_guard<std::mutex> guard{m_mutex};

    if (m_pending_writes.empty()) {
        m_write_timer.expires_from_now(boost::posix_time::milliseconds(100));
        m_write_timer.async_wait(boost::bind(&websocket_session::write_next_pending, shared_from_this()));
        return;
    }

    m_ws.async_write(net::buffer(m_pending_writes.front()), 
        beast::bind_front_handler(&websocket_session::on_write, shared_from_this()));
}

void rosweb::websocket_session::on_write(beast::error_code ec, std::size_t bytes_transferred) {
    if (m_session_closed) return;

    std::unique_lock<std::mutex> lock{m_mutex};

    m_pending_writes.pop();

    lock.unlock();

    write_next_pending();
}
