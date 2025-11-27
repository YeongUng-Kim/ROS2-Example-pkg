#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Bridge : public rclcpp::Node {
    public:
        Bridge(): Node("Bridge") {
            RCLCPP_INFO(this->get_logger(), "--- Start ---");
        }

        void setup(const rclcpp::Node::SharedPtr& src, const rclcpp::Node::SharedPtr& des){
            _subFsrc = src->create_subscription<std_msgs::msg::String>("/String", 10, bind(&Bridge::subCallback, this, std::placeholders::_1));

            _pub2des = des->create_publisher<std_msgs::msg::String>("/String", 10);
        }

        void subCallback(const std_msgs::msg::String& msg){
            _pub2des->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Receive a msg from [src]; Pub to [des]");
            RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub2des;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subFsrc;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::InitOptions options;

    options.set_domain_id(0);
    auto src_context = std::make_shared<rclcpp::Context>();
    src_context->init(argc, argv, options);
    rclcpp::NodeOptions srcOption = rclcpp::NodeOptions();
    srcOption.context(src_context);
    auto src = rclcpp::Node::make_shared("Domain00", srcOption);
    // auto src = std::make_shared<rclcpp::Node>("Domain00");

    options.set_domain_id(99);
    auto des_context = std::make_shared<rclcpp::Context>();
    des_context->init(argc, argv, options);
    rclcpp::NodeOptions desOption = rclcpp::NodeOptions();
    desOption.context(des_context);
    auto des = rclcpp::Node::make_shared("Domain99", desOption);

    auto bridge = Bridge();
    bridge.setup(src, des);

    auto src_exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    auto des_exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    src_exec->add_node(src);
    des_exec->add_node(des);

    std::thread threadSrc([=] {src_exec->spin(); } );
    std::thread threadDes([=] {des_exec->spin(); } );

    threadSrc.join();
    threadDes.join();

    rclcpp::shutdown();
    return 0;
}