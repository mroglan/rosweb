#ifndef ROSWEB_ERRORS_H
#define ROSWEB_ERRORS

#include <iostream>
#include <string>

namespace rosweb {
    namespace errors {
        void show_critical_error(const std::string& msg);

        void show_noncritical_error(const std::string& msg);

        class base_error : public std::exception {
            public:
                base_error(const std::string& msg);

                const char* what();

                void show() const;
            
            private:
                std::string m_msg;
        };

        class message_parse_error : public base_error {
            public:
                message_parse_error(const std::string& msg);

                void show() const;
        };

        class request_error : public base_error {
            public:
                request_error(const std::string& msg);

                void show() const;
        };
    }
}

#endif