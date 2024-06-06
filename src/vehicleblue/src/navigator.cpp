#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

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

      // subscriber code
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/model/vehicle_blue/odometry", 1, std::bind(&Navigator::topic_callback, this, _1));

    }

  private:
    void timer_callback()
    {
      float destination_x = 10.0, destination_y = 5.0;      

      // get distance between vehicle and destination
      float distance_x = destination_x - this->vehicle_x;
      float distance_y = destination_y - this->vehicle_y;

      // turning distance into polar coordinates
      float norm_distance = std::sqrt(std::pow(distance_x, 2) + std::pow(distance_y, 2));
      float distance_yaw = std::acos(distance_x/norm_distance);
      float yaw = distance_yaw - this->vehicle_yaw;

      // impose a speed limit
      if (norm_distance > 1.5){
        norm_distance = 1.5;
      }
      else if (norm_distance < 0.0001){
        norm_distance = 0.0;
      }

      // prepare and publish twist message
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = norm_distance;
      message.linear.y = 0.0;
      message.linear.z = 0.0;
      message.angular.x = 0.0;
      message.angular.y = 0.0;
      message.angular.z = yaw;
      
      RCLCPP_INFO(this->get_logger(), "Publishing: %f  %f", message.linear.x, message.angular.z);
      publisher_->publish(message);
    }

    void topic_callback(const nav_msgs::msg::Odometry & msg)
    {
      // convert quarternion to rpy
      double roll, pitch, yaw;

      tf2::Quaternion q(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w);

      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);

      // save vehicle position and orientation
      this->vehicle_x = msg.pose.pose.position.x;
      this->vehicle_y = msg.pose.pose.position.y;
      this->vehicle_yaw = yaw;

      RCLCPP_INFO(this->get_logger(), 
      "Saved Odometry: %f  %f  %f", this->vehicle_x, this->vehicle_y, this->vehicle_yaw);
    
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    size_t count_;

    float vehicle_x = 0.0, vehicle_y = 0.0, vehicle_yaw=0.0;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
}
