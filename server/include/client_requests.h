#ifndef ROSWEB_CLIENT_REQUESTS_H
#define ROSWEB_CLIENT_REQUESTS_H

#include <memory>
#include <mutex>
#include <condition_variable>

#include "./json.hpp"

namespace rosweb {
    class websocket_session;
}

namespace rosweb {
    namespace client_requests {
        // To help build the requests in client_requests
        namespace helpers {

            struct NavSat {
                double latitude;
                double longitude;
                double altitude;
            };

            struct Quaternion {
                double x;
                double y;
                double z;
                double w;
            };

            struct Waypoint {
                NavSat position;
                Quaternion orientation;
                int group;
            };
        }
    }
}

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

            struct destroy_subscriber_request : client_request {
                std::string topic_name;
                std::string msg_type;

                ~destroy_subscriber_request();
            };

            struct change_subscriber_request : client_request {
                std::string prev_topic_name;
                std::string new_topic_name;
                std::string msg_type;

                ~change_subscriber_request();
            };

            struct toggle_pause_subscriber_request : client_request {
                std::string topic_name;
                std::string msg_type;

                ~toggle_pause_subscriber_request();
            };

            struct bagged_image_to_video_request : client_request {
                std::string output_name;
                std::string bag_path;
                std::string topic_name;
                std::string encoding;
                bool create_html;

                ~bagged_image_to_video_request();
            };

            struct save_waypoints_request : client_request {
                std::vector<helpers::Waypoint> waypoints;
                std::vector<int> groups;
                std::string save_dir;

                ~save_waypoints_request();
            };

            class client_request_handler {
                public:
                    ~client_request_handler();

                    void handle_incoming_request(nlohmann::json_abi_v3_11_2::json& j,
                        std::shared_ptr<rosweb::websocket_session> session);

                    void acknowledge();

                    bool is_acknowledged() const;

                    const client_request* get_data() const;
                
                private:
                    mutable std::mutex m_mutex;
                    std::condition_variable m_cv;

                    bool m_acknowledged{true};
                    std::unique_ptr<client_request> m_data;

                    void handle_incoming_reset_request(nlohmann::json_abi_v3_11_2::json& j);

                    void handle_incoming_subscriber_request(nlohmann::json_abi_v3_11_2::json& j);

                    void handle_incoming_destroy_subscriber_request(nlohmann::json_abi_v3_11_2::json& j);

                    void handle_incoming_change_subscriber_request(nlohmann::json_abi_v3_11_2::json& j);

                    void handle_incoming_toggle_pause_subscriber_request(nlohmann::json_abi_v3_11_2::json& j);

                    void handle_incoming_bagged_image_to_video_request(nlohmann::json_abi_v3_11_2::json& j);

                    void handle_incoming_save_waypoints_request(nlohmann::json_abi_v3_11_2::json& j);
            };
    }
}

#endif