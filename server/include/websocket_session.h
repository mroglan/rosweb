#ifndef ROSWEB_WEBSOCKET_SESSION_H
#define ROSWEB_WEBSOCKET_SESSION_H

#include <memory>
#include <queue>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <condition_variable>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>

namespace rosweb {
    class bridge;
};

namespace rosweb {

    class websocket_session : public std::enable_shared_from_this<rosweb::websocket_session> {

        public:
            websocket_session(std::shared_ptr<rosweb::bridge> bridge, 
                boost::asio::ip::tcp::socket&& socket, boost::asio::io_context& ioc);

            ~websocket_session();
            
            void run();

            void read();

            void queue_messages(const std::vector<std::string>& msgs);

            bool is_closed();

        private:
            std::mutex m_mutex;
            std::condition_variable m_cv; 

            std::weak_ptr<rosweb::bridge> m_bridge;
            boost::beast::websocket::stream<boost::beast::tcp_stream> m_ws;
            boost::beast::flat_buffer m_buffer;

            boost::asio::deadline_timer m_write_timer;
            std::queue<std::string> m_pending_writes;

            bool m_session_closed{false};

            void on_accept(boost::beast::error_code ec);

            void on_read(boost::beast::error_code ec, std::size_t bytes_transferred);

            void write_next_pending();

            void on_write(boost::beast::error_code ec, std::size_t bytes_transferred);
    };
}

#endif