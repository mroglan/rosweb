#ifndef ROSWEB_CLIENT_REQUESTS_H
#define ROSWEB_CLIENT_REQUESTS_H

#include <memory>
#include <mutex>
#include <condition_variable>

#include "./json.hpp"

namespace rosweb {
    namespace client_requests {

            struct client_request {
                std::string operation;

                virtual ~client_request();
            };

            struct create_subscriber_request : client_request {
                std::string topic_name; 
                std::string msg_type;

                ~create_subscriber_request();
            };

            class client_request_handler {
                public:
                    ~client_request_handler();

                    void handle_incoming_request(nlohmann::json_abi_v3_11_2::json& j);

                    void acknowledge();

                    bool is_acknowledged() const;

                    const client_request* get_data() const;
                
                private:
                    mutable std::mutex m_mutex;
                    std::condition_variable m_cv;

                    bool m_acknowledged{true};
                    std::unique_ptr<client_request> m_data;

                    void handle_incoming_subscriber_request(nlohmann::json_abi_v3_11_2::json& j);
            };
    }
}

#endif