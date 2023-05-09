#include <iostream>
#include <string>

#include "../include/error.h"

namespace rosweb {
    namespace errors {
        void show_critical_error(const std::string& msg) {
            std::cerr << "ROSWEB CRITICAL ERROR:\n";
            std::cerr << msg << '\n';
        }
    }
}