#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <memory>
#include <chrono>
#include <functional>
#include <cmath>
#include <unistd.h>

#include "turtle/keyboard.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
//球员检测范围
#define d 0.75
#define d0 0.4
#define pi 3.1415926535
#define speed_max 3.0
#define speed_min 0.1
#define speed_default 1.0

bool turn_ = 0;  //1转   0不转
bool pause_game = 1;
double turn_ang = 0;
double theta = 0;
double x2,y2,x3,y3;
float ball_speed = speed_default;
int turtle2_scores = 0,turtle3_scores = 0;

class Cmd_vel_Publisher : public rclcpp::Node
{
    public:
        Cmd_vel_Publisher():rclcpp::Node("Cmd_vel_Publisher")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            timer_ = this->create_wall_timer(100ms, std::bind(&Cmd_vel_Publisher::timer_callback, this));
            
        }
    private:
        void timer_callback()
        {
            if(pause_game==0)
            {
                //publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
                auto twist = geometry_msgs::msg::Twist();
                if(turn_==1)
                {
                    twist.angular.z = turn_ang;
                    twist.linear.x = 0;
                    publisher_->publish(twist);
                    turn_ = 0;
                    sleep(1);
                }
                if(turn_!=1)
                {
                    twist.angular.z = 0;
                    twist.linear.x = ball_speed;
                    publisher_->publish(twist);
                }
                //timer_->cancel();   //取消定时器，只运行一次回调函数
            }
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};

class GetPose:public rclcpp::Node
{
    public:
        GetPose():rclcpp::Node("GetPose")
        {
            subscriber_2 = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&GetPose::sub_callback2, this, _1));
            subscriber_3 = this->create_subscription<turtlesim::msg::Pose>("/turtle3/pose", 10, std::bind(&GetPose::sub_callback3, this, _1));
            subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&GetPose::sub_callback, this, _1));

            publisher_quit = this->create_publisher<std_msgs::msg::Bool>("/quit_",10);
            subscription_pause = this->create_subscription<std_msgs::msg::Bool>("/pause", 10, std::bind(&GetPose::pause_callback, this, _1));
            subscription_speedup = this->create_subscription<std_msgs::msg::Bool>("/speedup", 10, std::bind(&GetPose::speedup_callback, this, _1));
            subscription_speeddown = this->create_subscription<std_msgs::msg::Bool>("/speeddown", 10, std::bind(&GetPose::speeddown_callback, this, _1));
            subscription_speedreset = this->create_subscription<std_msgs::msg::Bool>("/speedreset", 10, std::bind(&GetPose::speedreset_callback, this, _1));
            subscription_clearscore = this->create_subscription<std_msgs::msg::Bool>("/clearscore", 10, std::bind(&GetPose::clearscore_callback, this, _1));
        }

    private:
        void sub_callback2(const turtlesim::msg::Pose::SharedPtr pose) const
        {
            x2 = pose->x;
            y2 = pose->y;
        }
        void sub_callback3(const turtlesim::msg::Pose::SharedPtr pose) const
        {
            x3 = pose->x;
            y3 = pose->y;
        }
        void sub_callback(const turtlesim::msg::Pose::SharedPtr pose) const
        {   
            if(pause_game==0)
            {
                auto quit_ = std_msgs::msg::Bool();
                theta = pose->theta;
                double x = pose->x;
                double y = pose->y;
                if(x<=0)    //触碰左边界，turtle2得分
                {
                    RCLCPP_INFO(rclcpp::get_logger("Game"),"\n-----------------------\nGame over! Player1 win!\n目前比分： %d:%d\n-----------------------",++turtle2_scores,turtle3_scores);
                    quit_.data = 1;
                    publisher_quit->publish(quit_);
                    pause_game = 1;
                    //rclcpp::shutdown();
                    //exit(0);
                }
                else if(x>=11)    //触碰右边界，turtle1得分
                {
                    RCLCPP_INFO(rclcpp::get_logger("Game"),"\n-----------------------\nGame over! Player2 win!\n目前比分： %d:%d\n-----------------------",turtle2_scores,++turtle3_scores);
                    quit_.data = 1;
                    publisher_quit->publish(quit_);
                    pause_game = 1;
                    //rclcpp::shutdown();
                    //exit(0);
                }

                else if( 
                    (( x<(x2+d) && x>(x2+d0) )  &&  ( y>(y2-d0) && y<(y2+d0) ))
                    ||
                    (( x<(x3+d) && x>(x3+d0) )  &&  ( y>(y3-d0) && y<(y3+d0) ))
                )   //触碰球员右边
                {
                    if(theta<=-1.57 && theta>=-pi) //向下
                    {
                        turn_ = 1;  //左转
                        turn_ang = (-pi-theta*2);
                        sleep(1);
                    }
                    else if(theta>=1.57 && theta<=pi)  //向上
                    {
                        turn_ = 1;  //右转
                        turn_ang = (pi-theta*2);
                        sleep(1);
                    }
                }
                else if( 
                    (( x<(x2-d0) && x>(x2-d) )  &&  ( y>(y2-d0) && y<(y2+d0) ))
                    ||
                    (( x<(x3-d0) && x>(x3-d) )  &&  ( y>(y3-d0) && y<(y3+d0) ))
                )   //触碰球员左边
                {
                    if(theta>-1.57 && theta<=0) //向下
                    {
                        turn_ = 1;  //右转
                        turn_ang = (-pi-theta*2);
                        sleep(1);
                    }
                    else if(theta<1.57 && theta>0)  //向上
                    {
                        turn_ = 1;  //左转
                        turn_ang = (pi-theta*2);
                        sleep(1);
                    }
                }

                else if( y>=11 || 
                    (( y<(y2-d0) && y>(y2-d) )  &&  ( x>(x2-d0) && x<(x2+d0) ))
                    ||
                    (( y<(y3-d0) && y>(y3-d) )  &&  ( x>(x3-d0) && x<(x3+d0) ))
                )   //触碰上边界 或 触碰球员下边
                {
                    if(theta>1.57 && theta<=pi)  //向左
                    {
                        turn_ = 1; //左转
                        turn_ang = (pi*2-theta*2);
                        sleep(1);
                    }
                    else if(theta<=1.57 && theta>=0)  //向右
                    {
                        turn_ = 1;  //右转
                        turn_ang = (-theta*2);
                        sleep(1);
                    }
                }
                else if(y<=0 || 
                    (( y<(y2+d) && y>(y2+d0) )  &&  ( x>(x2-d0) && x<(x2+d0) ))
                    ||
                    (( y<(y3+d) && y>(y3+d0) )  &&  ( x>(x3-d0) && x<(x3+d0) ))
                )    //触碰下边界 或 触碰球员上边
                {
                    if(theta<-1.57 && theta>=-pi) //向左
                    {
                        turn_ = 1;   //右转
                        turn_ang = (-pi*2-theta*2);
                        sleep(1);
                    }
                    else if(theta>=-1.57 && theta <=0)
                    {
                        turn_ = 1; //左转
                        turn_ang = (-theta*2);
                        sleep(1);
                    }
                }
            }
        }

        void pause_callback(const std_msgs::msg::Bool::SharedPtr pause_)
        {
            if(pause_->data==0)
            {
                RCLCPP_INFO(rclcpp::get_logger("Game"),"开始！\n当前球速为%.1f",ball_speed);
                pause_game = 0;
            }
            else
            {   
                RCLCPP_INFO(rclcpp::get_logger("Game"),"暂停！");
                pause_game = 1;
            }
        }

        void speedup_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr speed_up)
        {
            if(ball_speed<speed_max-0.1)
            {
                ball_speed+=0.1;
                RCLCPP_INFO(rclcpp::get_logger("调速系统"),"球速上升0.1，当前球速为%.1f。",ball_speed);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("调速系统"),"球度已经达到上限%.1f了，不能在升高了！",speed_max);
            }
        }
        void speeddown_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr speed_down)
        {
            if(ball_speed>speed_min)
            {
                ball_speed-=0.1;
                RCLCPP_INFO(rclcpp::get_logger("调速系统"),"球速下降0.1，当前球速为%.1f。",ball_speed);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("调速系统"),"球速已经达到下限%.1f了，不能在降低了！",speed_min);
            }
        }
        void speedreset_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr speed_reset)
        {
                ball_speed=speed_default;
                RCLCPP_INFO(rclcpp::get_logger("调速系统"),"重置球速为%.1f。",speed_default);
        }

        void clearscore_callback([[maybe_unused]] const std_msgs::msg::Bool::SharedPtr clear_score)
        {
            RCLCPP_WARN(rclcpp::get_logger("Game"),"清除当前比分！");
            turtle2_scores = 0;
            turtle3_scores = 0;
        }

        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_2;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_3;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_quit;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_pause;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_speedup;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_speeddown;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_speedreset;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_clearscore;
        
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<Cmd_vel_Publisher>();
    auto node2 = std::make_shared<GetPose>();
    auto node3 = std::make_shared<TeleopKeyNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.add_node(node2);
    executor.add_node(node3);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}