#ifndef ROSWEB_SERVER_RESPONSES_H
#define ROSWEB_SERVER_RESPONSES_H

#include <string>
#include "./json.hpp"

namespace rosweb {
    namespace server_responses {

        class standard {
            public:
                void set_operation(const std::string& operation);

                void set_status(int status);

                void set_msg(const std::string& msg);

                nlohmann::json_abi_v3_11_2::json json();

                std::string stringify();
            
            private:
                std::string m_operation;
                int m_status;
                std::string m_msg;
        };
    }
}

#endif