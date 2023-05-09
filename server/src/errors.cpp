#include <iostream>
#include <string>

#include "../include/error.h"

namespace rosweb {
    namespace errors {
        void show_critical_error(const std::string& msg) {
            std::cerr << "\033[1;31m" << "ROSWEB CRITICAL ERROR:\n";
            std::cerr << msg << "\033[0m\n";
        }

        void show_noncritical_error(const std::string& msg) {
            std::cerr << "\033[1;31m" << "ROSWEB ERROR:\n";
            std::cerr << msg << "\033[0m\n";
        }
    }
}