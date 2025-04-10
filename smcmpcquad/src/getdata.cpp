#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>
#include <fstream>

using namespace Eigen;

class Getdata : public rclcpp::Node
{
public:
  Getdata() : Node("getdata_node"), last_time_(this->now()), U(VectorXd::Zero(4)) // Initialize U with size 4
  {
    control_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/controllaw", 10, std::bind(&Getdata::controlCallback, this, std::placeholders::_1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/droneposition/odom", 10, std::bind(&Getdata::topic_callback, this, std::placeholders::_1));

    // Open file in append mode
    outfile_.open("dataahsmcfailure.txt", std::ios::app);
    if (!outfile_.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open file for writing.");
    }
  }

  ~Getdata()
  {
    if (outfile_.is_open())
    {
      outfile_.close();
    }
  }

private:
  void controlCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4)
    {
      RCLCPP_WARN(this->get_logger(), "Expected 4 control inputs, but got %zu", msg->data.size());
      return;
    }

    for (size_t i = 0; i < 4; ++i)
    {
      U(i) = msg->data[i];
    }
  }

  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;
    const auto &linear_velocity = msg->twist.twist.linear;
    const auto &angular_velocity = msg->twist.twist.angular;

    // Convert quaternion to roll, pitch, and yaw
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 rpy(q);
    double roll, pitch, yaw;
    rpy.getRPY(roll, pitch, yaw);

    double x = position.x;
    double y = position.y;
    double z = position.z;
    double phi = roll;     // Roll angle
    double theta = pitch;  // Pitch angle
    double psi = yaw;      // Yaw angle
    double xd = linear_velocity.x;
    double yd = linear_velocity.y;
    double zd = linear_velocity.z;
    double phid = angular_velocity.x;
    double thetad = angular_velocity.y;
    double psid = angular_velocity.z;

    auto now = this->now(); // Use node's clock
    auto elapsed_time_ms = now.nanoseconds() / 1'000'000; // Convert nanoseconds to milliseconds

    if (outfile_.is_open())
    {
      outfile_ << "Time: " << elapsed_time_ms
               << ", x: " << x << ", y: " << y << ", z: " << z
               << ", xd: " << xd << ", yd: " << yd << ", zd: " << zd
               << ", phi: " << phi << ", theta: " << theta << ", psi: " << psi
               << ", U1: " << U(0) << ", U2: " << U(1) << ", U3: " << U(2) << ", U4: " << U(3)
               << "\n";
    }
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_sub_;
  VectorXd U;
  rclcpp::Time last_time_;
  std::ofstream outfile_; // File stream object
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Getdata>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
