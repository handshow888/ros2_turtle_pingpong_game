#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>
#include "turtlesim/srv/set_pen.hpp"
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <random>
#include <unistd.h>
using namespace std::chrono_literals;
using std::placeholders::_1;

class Spawn : public rclcpp::Node
{
public:
    Spawn() : Node("spawn_node")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>("/start_game",10,std::bind(&Spawn::callback, this, _1));
    }
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_spawn = this->create_client<turtlesim::srv::Spawn>("/spawn"); //call /spawn
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_kill = this->create_client<turtlesim::srv::Kill>("/kill"); //call /kill
private:
    void callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr start_game)
    {   
        // 处理接收到的消息
        std::mt19937 gen(std::random_device{}());                // 创建一个伪随机数生成器
        std::uniform_real_distribution<double> distr(0.0, 6.28); // 创建一个均匀分布函数
        double randomx = distr(gen);                             // 生成一个随机浮点数
        // 创建客户端并发送请求
        /*kill*/
        RCLCPP_INFO(this->get_logger(), "kill!");
        //auto client_kill = create_client<turtlesim::srv::Kill>("/kill");
        auto kill1 = std::make_shared<turtlesim::srv::Kill::Request>();
        kill1->name = "turtle1";
        auto kill2 = std::make_shared<turtlesim::srv::Kill::Request>();
        kill2->name = "turtle2";
        auto kill3 = std::make_shared<turtlesim::srv::Kill::Request>();
        kill3->name = "turtle3";

        auto future_kill1 = client_kill->async_send_request(kill1);
        auto future_kill2 = client_kill->async_send_request(kill2);
        auto future_kill3 = client_kill->async_send_request(kill3);
        usleep(100000);   //暂停10000微妙=100毫秒
        /*spawn*/
        RCLCPP_INFO(this->get_logger(), "spawn!");
        //auto client_spawn = create_client<turtlesim::srv::Spawn>("/spawn");
        auto spawn_request1 = std::make_shared<turtlesim::srv::Spawn::Request>(); // turtle1 position
        spawn_request1->x = 5.5;
        spawn_request1->y = 5.5;
        spawn_request1->theta = randomx;
        spawn_request1->name = "turtle1";
        auto spawn_request2 = std::make_shared<turtlesim::srv::Spawn::Request>(); // turtle2 position
        spawn_request2->x = 2;
        spawn_request2->y = 2;
        spawn_request2->name = "turtle2";
        auto spawn_request3 = std::make_shared<turtlesim::srv::Spawn::Request>(); // turtle3 position
        spawn_request3->x = 9;
        spawn_request3->y = 9;
        spawn_request3->theta = 3.14;
        spawn_request3->name = "turtle3";

        auto future_spawn1 = client_spawn->async_send_request(spawn_request1);
        auto future_spawn2 = client_spawn->async_send_request(spawn_request2);
        auto future_spawn3 = client_spawn->async_send_request(spawn_request3);
        RCLCPP_INFO(this->get_logger(), "spawn处理完成!");
    }
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;

};
class Set_Pen_Off:public rclcpp::Node
{
    public:
        Set_Pen_Off():Node("Set_Pen_Off")
        {
            timer_ = this->create_wall_timer(1ms, std::bind(&Set_Pen_Off::set_pen_off, this));
        }
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_set_pen1 = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_set_pen2 = this->create_client<turtlesim::srv::SetPen>("/turtle2/set_pen");
        rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr client_set_pen3 = this->create_client<turtlesim::srv::SetPen>("/turtle3/set_pen");
    private:
        void set_pen_off()
        {
            auto request_set_pen1 = std::make_shared<turtlesim::srv::SetPen::Request>();
            request_set_pen1->off = 1;
            auto future_set_pen1 = client_set_pen1->async_send_request(request_set_pen1);
            auto request_set_pen2 = std::make_shared<turtlesim::srv::SetPen::Request>();
            request_set_pen2->off = 1;
            auto future_set_pen2 = client_set_pen2->async_send_request(request_set_pen2);
            auto request_set_pen3 = std::make_shared<turtlesim::srv::SetPen::Request>();
            request_set_pen3->off = 1;
            auto future_set_pen3 = client_set_pen3->async_send_request(request_set_pen3);
        }
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Spawn>();
    auto node2 = std::make_shared<Set_Pen_Off>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
