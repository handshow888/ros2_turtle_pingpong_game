#include <iostream>
#include <string>
#include <chrono>
#include <unistd.h>
#include <termios.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;

// Ascii codes for small keyboard characters
// https://en.wikipedia.org/wiki/ASCII
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_W 0x77
#define KEYCODE_S 0x73
#define KEYCODE_A 0x61
#define KEYCODE_B 0x62
#define KEYCODE_C 0x63
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_P 0x70
#define KEYCODE_Q 0x71
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_V 0x76
#define KEYCODE_I 0x69
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C

/**
 * @brief 基于Linux系统下的键盘监听
 */
class KeyBoardReader
{
private:
    termios new_settings{};
    termios stored_settings{};
    int kfd{0}; // 标准输入文件句柄

public:
    KeyBoardReader()
    {
        // 获取标准输入的终端属性，并存储在 stored_settings 结构体中
        tcgetattr(0, &stored_settings);
        // 复制 stored_settings 的内容到 new_settings 结构体中
        memcpy(&new_settings, &stored_settings, sizeof(termios));
        // 屏蔽整行缓存和本地回显功能，设置新行符、文件结束符
        new_settings.c_lflag &= ~(ICANON | ECHO);
        new_settings.c_cc[VEOL] = 1;
        new_settings.c_cc[VEOF] = 2;
        // 将标准输入的终端属性设置为 new_settings 的值，实现键盘监听
        tcsetattr(kfd, TCSANOW, &new_settings);
    }

    ~KeyBoardReader()
    {
        // 恢复标准输入终端属性
        shutdown();
    }

    void reader(char *c)
    {
        int rc = read(kfd, c, 1);

        // 读取失败
        if (rc < 0)
        {
            // 抛出异常
            throw std::runtime_error("读取键盘失败");
        }
    }

    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &stored_settings);
    }

    termios getStoredSettings()
    {
        return stored_settings;
    }
};  

/**
 * @brief 键盘监听节点，仅供参考
 */
class TeleopKeyNode : public rclcpp::Node
{
public:
    KeyBoardReader input;
    TeleopKeyNode() : rclcpp::Node("TeleopKey")
    {
        // 无限循环，用来监听，但会阻塞该节点的主线程
        // 可以创建线程，用线程来监听，这样可以不阻塞主线程
        // 具体的线程创建方法请自行查阅资料
        publisher_2 = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        publisher_3 = this->create_publisher<geometry_msgs::msg::Twist>("/turtle3/cmd_vel", 10);
        timer_ = this->create_wall_timer(1ms, std::bind(&TeleopKeyNode::timer_callback, this));
        subscription_quit = this->create_subscription<std_msgs::msg::Bool>("/quit_", 10, std::bind(&TeleopKeyNode::quit_callback, this, _1));
        publisher_start = this->create_publisher<std_msgs::msg::Bool>("/start_game", 10);
        publisher_pause = this->create_publisher<std_msgs::msg::Bool>("/pause", 10);
        publisher_speedup = this->create_publisher<std_msgs::msg::Bool>("/speedup", 10);
        publisher_speeddown = this->create_publisher<std_msgs::msg::Bool>("/speeddown", 10);
        publisher_speedreset = this->create_publisher<std_msgs::msg::Bool>("/speedreset", 10);
        publisher_clearscore = this->create_publisher<std_msgs::msg::Bool>("/clearscore", 10);
        ifpause = 0;
    }
    bool ifpause;
    ~TeleopKeyNode()
    {
        input.shutdown();
    }

    void getKeyPress(char* c)
    {
        try
        {
            input.reader(c);
        }
        catch (const std::runtime_error &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    void quit()
    {
        input.shutdown();
        rclcpp::shutdown();
        exit(0);
    }

private:
    void timer_callback()
        {
            auto twist = geometry_msgs::msg::Twist();
            auto start = std_msgs::msg::Bool();
            auto pause__ = std_msgs::msg::Bool();
            auto bool_true = std_msgs::msg::Bool();
            bool_true.data = 1;
            char c;
            getKeyPress(&c);
            // c代表者按下键盘时的字符，通过c的值就可以确认按下的是哪个键
            //std::cout << c << '\n';
            // 一定要设置退出键，否则会无法退出，这里是q键
            if (c == KEYCODE_Q)
            {
                // 退出时一定要恢复控制台的标准输入，否则退出程序后控制台将无法输入任何字符
                RCLCPP_INFO(rclcpp::get_logger("Game"),"\n-----------------------\n手动退出游戏！\n-----------------------");
                quit();
            }
            // 在下面可以使用switch或if来对应每个键的作用
            switch (c)
            {
            //WSAD控制turtle2
            case KEYCODE_W:
                twist.linear.x = 2;
                twist.angular.z = 0;
                publisher_2->publish(twist);
                break;
            case KEYCODE_S:
                twist.linear.x = -2;
                twist.angular.z = 0;
                publisher_2->publish(twist);
                break;
            case KEYCODE_A:
                twist.linear.x = 0;
                twist.angular.z = 1.57;
                publisher_2->publish(twist);
                break;
            case KEYCODE_D:
                twist.linear.x = 0;
                twist.angular.z = -1.57;
                publisher_2->publish(twist);
                break;
            //IKJL控制turtle3
            case KEYCODE_I:
                twist.linear.x = 2;
                twist.angular.z = 0;
                publisher_3->publish(twist);
                break;
            case KEYCODE_K:
                twist.linear.x = -2;
                twist.angular.z = 0;
                publisher_3->publish(twist);
                break;
            case KEYCODE_J:
                twist.linear.x = 0;
                twist.angular.z = 1.57;
                publisher_3->publish(twist);
                break;
            case KEYCODE_L:
                twist.linear.x = 0;
                twist.angular.z = -1.57;
                publisher_3->publish(twist);
                break;
            //
            case KEYCODE_R:     //开始or重置游戏
                start.data = 1;
                publisher_start->publish(start);
                //usleep(500000);
                pause__.data = 0;
                publisher_pause->publish(pause__);
                break;
            case KEYCODE_P: //暂停or继续游戏
                if(ifpause==0)
                {
                    pause__.data = 1;
                    ifpause = 1;
                }
                else
                {
                    pause__.data = 0;
                    ifpause = 0;
                }
                publisher_pause->publish(pause__);
                break;
            case KEYCODE_T: //加速键
                publisher_speedup->publish(bool_true);
                break;
            case KEYCODE_G: //减速键
                publisher_speeddown->publish(bool_true);
                break;
            case KEYCODE_B: //重置球速键
                publisher_speedreset->publish(bool_true);
                break;
            case KEYCODE_C: //清除比分
                publisher_clearscore->publish(bool_true);
                break;
            default:
                break;
            }
        
        }
    void quit_callback(const std_msgs::msg::Bool::SharedPtr quit_)
    {
        if(quit_->data == 1)
        {
            input.shutdown();
            rclcpp::shutdown();
            exit(0);
        }
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_2;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_3;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_quit;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_start;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_pause;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_speedup;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_speeddown;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_speedreset;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_clearscore;
};

/*
int main(int argc,char** argv)
{   
    rclcpp::init(argc,argv);

    auto node = std::make_shared<TeleopKeyNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    
    return 0;
}
*/