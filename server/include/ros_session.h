#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace rosweb {
    class bridge;
}

namespace rosweb {
    class ros_session : public rclcpp::Node {
        public:
            ros_session(std::shared_ptr<rosweb::bridge> bridge);
        
        private:
            std::shared_ptr<rosweb::bridge> m_bridge;

            rclcpp::TimerBase::SharedPtr m_timer;

            void timer_callback();
    };
}