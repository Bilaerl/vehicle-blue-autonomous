#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Navigator : public rclcpp::Node
{
  public:
    Navigator() : Node("navigator"), count_(0)
    {
      // publisher code
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 2);
      timer_ = this->create_wall_timer(500ms, std::bind(&Navigator::timer_callback, this));

    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = 0.5;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = 0.0;
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.linear.x);
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
