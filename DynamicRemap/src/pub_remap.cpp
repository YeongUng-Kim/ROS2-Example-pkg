#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MWE : public rclcpp::Node {
    public:
        MWE(): Node("mwe"){
            RCLCPP_INFO(this->get_logger(), "--- Start ---");
            _timer = this->create_wall_timer(1000ms, std::bind(&MWE::timer_cb, this));
        }
    
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
        rclcpp::TimerBase::SharedPtr _timer;

        int _count=0;

        void timer_cb(){
            _publisher.reset();
            _publisher = this->create_publisher<std_msgs::msg::String>("Topic"+std::to_string(_count++ % 3),10);
            auto msg = std_msgs::msg::String();
            msg.data = "Hello, World! " + std::to_string(_count);
            _publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MWE>());
    rclcpp::shutdown();
    return 0;
}