#ifndef ROSWEB_ERRORS_H
#define ROSWEB_ERRORS

#include <iostream>
#include <string>

namespace rosweb {
    namespace errors {
        void show_critical_error(const std::string& msg);

        void show_noncritical_error(const std::string& msg);

        class message_parse_error : public std::exception {
            public:
                message_parse_error(char* msg);

                char* what();

                void show() const;
            
            private:
                char* m_msg;
        };
    }
}

#endif