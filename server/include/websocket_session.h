#ifndef ROSWEB_WEBSOCKET_SESSION_H
#define ROSWEB_WEBSOCKET_SESSION_H

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

namespace rosweb {
    class bridge;
};

namespace rosweb {

    class websocket_session : public std::enable_shared_from_this<rosweb::websocket_session> {

        public:
            websocket_session(std::shared_ptr<rosweb::bridge> bridge, 
                boost::asio::ip::tcp::socket&& socket);
            
            void run();

        private:
            std::mutex m_mutex;
            std::condition_variable m_cv; 

            std::shared_ptr<rosweb::bridge> m_bridge;
            boost::beast::websocket::stream<boost::beast::tcp_stream> m_ws;
            boost::beast::flat_buffer m_buffer;

            bool m_is_writing{false};

            void on_accept(boost::beast::error_code ec);
    };
}

#endif