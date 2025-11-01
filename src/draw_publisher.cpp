#include <chrono>
#include <utility>
#include <functional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DrawPublisher : public rclcpp::Node
{
public:
    DrawPublisher() : Node("draw_publisher"), time_(0)
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50ms),
                                         std::bind(&DrawPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        time_ += 0.01f;

        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x += time_;
        msg.angular.z = 2;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "Publishing: t=%.2f linear.x=%.2f angular.z=%.2f",
                    time_,
                    msg.linear.x,
                    msg.angular.z);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    float time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DrawPublisher>());
    rclcpp::shutdown();
    return 0;
}