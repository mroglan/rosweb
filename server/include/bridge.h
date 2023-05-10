#ifndef ROSWEB_BRIDGE_H
#define ROSWEB_BRIDGE_H

#include <memory>
#include <boost/beast/core.hpp>


namespace rosweb {
    class websocket_session;
    namespace client_requests {
        class client_request_handler;
    }
}

namespace rosweb {
    class bridge : public std::enable_shared_from_this<rosweb::bridge> {

        public:
            bridge();

            ~bridge();

            void run();

            void handle_incoming_ws_msg(const std::string& msg);

            std::shared_ptr<rosweb::client_requests::client_request_handler> get_client_request_handler();

        private:
            boost::asio::io_context m_ioc;         
            boost::asio::ip::tcp::acceptor m_acceptor;

            std::shared_ptr<rosweb::websocket_session> m_session;
            std::shared_ptr<rosweb::client_requests::client_request_handler> m_client_request_handler;

            void accept();
            void on_accept(boost::beast::error_code ec, boost::asio::ip::tcp::socket socket);
    };
}

#endif