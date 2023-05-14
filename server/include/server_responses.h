#ifndef ROSWEB_SERVER_RESPONSES_H
#define ROSWEB_SERVER_RESPONSES_H

#include <string>
#include "./json.hpp"

namespace rosweb {
    namespace server_responses {

        class standard {
            public:
                virtual ~standard();

                void set_operation(const std::string& operation);

                void set_status(int status);

                void set_msg(const std::string& msg);

                nlohmann::json_abi_v3_11_2::json json() const;

                virtual std::string stringify() const;

                bool operator!() const;
            
            private:
                std::string m_operation;
                int m_status;
                std::string m_msg;
        };

        class create_or_destroy_sub : public standard {
            public:
                virtual ~create_or_destroy_sub();

                void set_topic_name(const std::string& topic_name);

                void set_msg_type(const std::string& msg_type);

                std::string stringify() const override;
            
            private:
                std::string m_topic_name;
                std::string m_msg_type;
        };

        class change_sub : public standard {
            public:
                ~change_sub();

                void set_new_topic_name(const std::string& topic_name);

                void set_prev_topic_name(const std::string& topic_name);

                void set_msg_type(const std::string& msg_type);

                std::string stringify() const override;

            private:
                std::string m_prev_topic_name;
                std::string m_new_topic_name;
                std::string m_msg_type;
        };
    }
}

#endif