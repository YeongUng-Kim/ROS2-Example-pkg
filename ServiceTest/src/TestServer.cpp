#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class TestServer : public rclcpp::Node
{
public:
    TestServer() : Node("TestServer"), cnt_(0)
    {
        // 파라미터 선언 및 가져오기
        this->declare_parameter<std::string>("name", "default");
        srv_name_ = this->get_parameter("name").as_string();

        // Reentrant Callback Group 생성 (병렬 실행 허용)
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // 서비스 생성
        srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/Trigger",
            std::bind(&TestServer::serve_trigger, this, _1, _2),
            rmw_qos_profile_services_default,
            cb_group_
        );

        // 타이머 생성 (2초 주기)
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&TestServer::timer_callback, this),
            cb_group_
        );

        RCLCPP_INFO(this->get_logger(), "TestServer has been started with name: %s", srv_name_.c_str());
    }

private:
    // 타이머 콜백
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Alive");
    }

    // 서비스 콜백 (async 방식은 C++에서 기본적으로 스레드 풀에 의해 처리됨)
    void serve_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // 사용하지 않는 매개변수 경고 방지

        int current_cnt = cnt_.load();
        response->success = true;
        response->message = srv_name_ + "_" + std::to_string(current_cnt);

        RCLCPP_INFO(this->get_logger(), "Received request");
        RCLCPP_INFO(this->get_logger(), "Sleep in %d sec", current_cnt);

        // Python의 time.sleep(self.cnt)와 동일한 동작
        std::this_thread::sleep_for(std::chrono::seconds(current_cnt));

        RCLCPP_INFO(this->get_logger(), "Send response: %s", response->message.c_str());
        
        // 카운트 증가
        cnt_++;
    }

    // 멤버 변수
    std::string srv_name_;
    std::atomic<int> cnt_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TestServer>();

    // MultiThreadedExecutor 설정 (쓰레드 4개)
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}