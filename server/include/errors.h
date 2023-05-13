#ifndef ROSWEB_ERRORS_H
#define ROSWEB_ERRORS_H

#include <iostream>
#include <string>

namespace rosweb {
    namespace errors {
        void show_critical_error(const std::string& msg);

        void show_noncritical_error(const std::string& msg);

        class base_error : public std::exception {
            public:
                base_error(const std::string& msg);

                virtual ~base_error();

                const char* what();

                const std::string& get_msg() const;

                virtual void show() const;
            
            private:
                std::string m_msg;
        };

        class message_parse_error : public base_error {
            public:
                message_parse_error(const std::string& msg);

                ~message_parse_error();

                void show() const override;
        };

        class request_error : public base_error {
            public:
                request_error(const std::string& msg);

                ~request_error();

                void show() const override;
        };
    }
}

#endif