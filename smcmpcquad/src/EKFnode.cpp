#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>

using namespace Eigen;

class EKFNode : public rclcpp::Node
{
public:
  EKFNode(): Node("ekf_node"), Q(MatrixXd::Identity(12, 12) * 1e-5), R(MatrixXd::Identity(12, 12) * 1e-6), m(1.85), g(9.80),
        Kdx(0.0000267), Kdy(0.0000267), Kdz(0.0000625), Ixx(0.0785), Iyy(0.0785), Izz(0.105),
        Pk(MatrixXd::Zero(12, 12)), xkh(VectorXd::Zero(12)), U(VectorXd::Zero(4)), dt(0.005), last_time_(this->now())
  {
    // Create publishers and subscriptions
    ekf_state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ekf_state", 10);
    control_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/controllaw", 10, std::bind(&EKFNode::controlCallback, this, std::placeholders::_1));
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/droneposition/odom", 10, std::bind(&EKFNode::topic_callback, this, std::placeholders::_1));
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
    // Extract position and orientation
    const auto &position = msg->pose.pose.position;
    const auto &orientation = msg->pose.pose.orientation;
    const auto &linear_velocity = msg->twist.twist.linear;
    const auto &angular_velocity = msg->twist.twist.angular;

    // Convert quaternion to roll, pitch, and yaw
    tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf2::Matrix3x3 rpy(q);
    double roll, pitch, yaw;
    rpy.getRPY(roll, pitch, yaw);

    double xs = position.x;
    double ys = position.y;
    double zs = position.z;
    double phis = roll;  // Roll angle
    double thetas = pitch;  // Pitch angle
    double psis = yaw;  // Yaw angle
    double xsd = linear_velocity.x;
    double ysd = linear_velocity.y;
    double zsd = linear_velocity.z;
    double phisd = angular_velocity.x;
    double thetasd = angular_velocity.y;
    double psisd = angular_velocity.z;
    
    auto current_time = this->now();
    dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double x = xkh(0);
    double y = xkh(1);
    double z = xkh(2);
    double phi = xkh(3);
    double theta = xkh(4);
    double psi = xkh(5);
    double xd = xkh(6);
    double yd = xkh(7);
    double zd = xkh(8);
    double phid = xkh(9);
    double thetad = xkh(10);
    double psid = xkh(11);
    
    // Extract state variables
    VectorXd state_(12);
    state_ << position.x, position.y, position.z, roll, pitch, yaw,
        linear_velocity.x, linear_velocity.y, linear_velocity.z,
        angular_velocity.x, angular_velocity.y, angular_velocity.z;

    // State-transition matrix A
    MatrixXd A = MatrixXd::Zero(12, 12);
    A.block<6, 6>(0, 6) = MatrixXd::Identity(6, 6);
    A(6, 6) = -Kdx / m;
    A(7, 7) = -Kdy / m;
    A(8, 8) = -Kdz / m;
    A(9, 10) = psid * (Iyy - Izz) / Ixx;
    A(9, 11) = thetasd * (Iyy - Izz) / Ixx;
    A(10, 9) = -psid * (Ixx - Izz) / Iyy;
    A(10, 11) = -phid * (Ixx - Izz) / Iyy;
    A(11, 9) = thetasd * (Ixx - Iyy) / Izz;
    A(11, 10) = phid * (Ixx - Iyy) / Izz;

    // Declare and initialize B matrix
    MatrixXd B(12, 4);
    B.setZero();
    B(6, 0) = (sin(phis) * sin(psis) + cos(phis) * cos(psis) * sin(thetas)) / m;
    B(7, 0) = -(cos(psis) * sin(phis) - cos(phis) * sin(psis) * sin(thetas)) / m;
    B(8, 0) = cos(phis) * cos(thetas) / m;
    B(9, 1) = 1 / Ixx;
    B(10, 2) = 1 / Iyy;
    B(11, 3) = 1 / Izz;

    // Prediction step
    VectorXd xkhm = xkh + (A * xkh + B * U) * dt;
    MatrixXd Pkm = A * Pk * A.transpose() + Q;

    // Kalman gain
    MatrixXd H = MatrixXd::Identity(12, 12);
    MatrixXd Kk = Pkm * H.transpose() * (H * Pkm * H.transpose() + R).inverse();

    // Update step
    xkh = xkhm + Kk * (state_ - H * xkhm);
    Pk = (MatrixXd::Identity(12, 12) - Kk * H) * Pkm;

    // Publish updated state
    std_msgs::msg::Float64MultiArray ekf_state;
    ekf_state.data.assign(xkh.data(), xkh.data() + xkh.size());
    ekf_state_publisher_->publish(ekf_state);

    RCLCPP_INFO(this->get_logger(), "EKF states: [%f, %f, %f, %f, %f, %f]", xkh(0), xkh(1), xkh(2), xkh(3), xkh(4), xkh(5));
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ekf_state_publisher_;

  VectorXd xkh;
  VectorXd U;
  MatrixXd Pk, Q, R;
  double m, g, Kdx, Kdy, Kdz, Ixx, Iyy, Izz, dt;
  rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EKFNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
