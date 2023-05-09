#ifndef ROSWEB_ERRORS_H
#define ROSWEB_ERRORS

#include <iostream>
#include <string>

namespace rosweb {
    namespace errors {
        void show_critical_error(const std::string& msg);
    }
}

#endif