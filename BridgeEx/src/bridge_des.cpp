#include <thread>
#include <string>
#include <cstdlib> // For std::getenv
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Bridge : public rclcpp::Node {
    public:
        Bridge(const std::string & nodeName, const rclcpp::NodeOptions & options): Node(nodeName, options) {
            RCLCPP_INFO(this->get_logger(), "--- Start ---");
        }

        void setup(const rclcpp::Node::SharedPtr & des){
            // outward
            _subFsrc = this->create_subscription<std_msgs::msg::String>("/Sub/String", 10, bind(&Bridge::subCallback, this, std::placeholders::_1));
            _pub2des = des->create_publisher<std_msgs::msg::String>("/String", 10);

            // inward
            _subFdes = des->create_subscription<std_msgs::msg::String>("/String", 10, bind(&Bridge::pubCallback, this, std::placeholders::_1));
            _pub2src = this->create_publisher<std_msgs::msg::String>("/Pub/String", 10);
        }

        void subCallback(const std_msgs::msg::String & msg){
            _pub2des->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Receive a msg from [src]; Pub to [des]");
            RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
        }

        void pubCallback(const std_msgs::msg::String & msg){
            _pub2src->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Receive a msg from [des]; Pub to [src]");
            RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
        }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub2des;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subFsrc;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub2src;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subFdes;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    auto src_context = std::make_shared<rclcpp::Context>();
    src_context->init(argc, argv);
    rclcpp::NodeOptions srcOption = rclcpp::NodeOptions();
    srcOption.context(src_context);
    // auto src = rclcpp::Node::make_shared("CurrentDomain", srcOption);
    auto bridge = std::make_shared<Bridge>("Bridge", srcOption);

    const char* domain_id_str = std::getenv("ROS_DOMAIN_ID");
    std::string str(domain_id_str ? domain_id_str : "Null");

    uint8_t domain = 99;
    rclcpp::InitOptions options;
    options.set_domain_id(domain);
    auto des_context = std::make_shared<rclcpp::Context>();
    des_context->init(argc, argv, options);
    rclcpp::NodeOptions desOption = rclcpp::NodeOptions();
    desOption.context(des_context);
    auto des = rclcpp::Node::make_shared("Bridge_" + str, desOption);

    bridge->setup(des);

    auto src_exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();
    auto des_exec = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>();

    src_exec->add_node(bridge);
    des_exec->add_node(des);

    std::thread threadSrc([=] {src_exec->spin(); } );
    std::thread threadDes([=] {des_exec->spin(); } );

    threadSrc.join();
    threadDes.join();

    rclcpp::shutdown();
    return 0;
}