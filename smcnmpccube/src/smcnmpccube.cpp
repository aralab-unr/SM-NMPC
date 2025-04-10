#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <fstream>

using namespace std::chrono_literals;
#define NX          ACADO_NX    
#define NXA         ACADO_NXA   
#define NU          ACADO_NU   
#define N           ACADO_N   
#define NOD         ACADO_NOD   
#define NY          ACADO_NY   
#define NYN         ACADO_NYN  
#define NUM_STEPS   1         
#define VERBOSE     1           

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

template <typename T>
T clamp(T value, T min, T max) {
    return (value < min) ? min : (value > max) ? max : value;
}

class PIDXY {
public:
    PIDXY(double kpx, double kdx, double kpy, double kdy) 
        : kpx_(kpx), kdx_(kdx), kpy_(kpy), kdy_(kdy), 
          previous_error_x(0), previous_error_y(0) {}

    std::pair<double, double> calculatexy(double setpointx, double setpointy, double pvx, double pvy, double pvphi, double pvtheta, double psi, double dt) {
        double errorx = setpointx - pvx;
        double errory = setpointy - pvy;
        double derivativex = (dt > 1e-5) ? (errorx - previous_error_x) / dt : 0; 
        double derivativey = (dt > 1e-5) ? (errory - previous_error_y) / dt : 0;

        previous_error_x = errorx;
        previous_error_y = errory;

        double thetade = kpx_ * errorx + kdx_ * derivativex;
        double phide = -kpy_ * errory - kdy_ * derivativey;
        double phir = phide * cos(psi) + thetade * sin(psi);
        double thetar = -phide * sin(psi) + thetade * cos(psi);
        phir = clamp(phir, -0.1, 0.1);
        thetar = clamp(thetar, -0.1, 0.1);

        return {phir, thetar};
    }

private:
    double kpx_, kdx_, kpy_, kdy_;
    double previous_error_x, previous_error_y;
};

class NMPCSMCCUBE : public rclcpp::Node {
public:
    NMPCSMCCUBE(): Node("NMPCSMCCUBE"), controlxy_(0.05, 0.1, 0.05, 0.1), xr(5.0), yr(5.0), zr(5.0), psir(0), x(0), y(0), z(0), xd(0), yd(0), zd(0), phi(0), theta(0), psi(0), phid(0), thetad(0), psid(0), w9(0), w10(0), addforce(0), pretime_(this->get_clock()->now()) {
        propvel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/prop_vel", 10);
        trajectory_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/trajectory", 10, std::bind(&NMPCSMCCUBE::trajectoryCallback, this, std::placeholders::_1));
        sideprop_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/sidepropvel", 10, std::bind(&NMPCSMCCUBE::sidepropCallback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/droneposition/odom", 10, std::bind(&NMPCSMCCUBE::stateCallback, this, std::placeholders::_1));
    }

private:

    void trajectoryCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 3) {
            RCLCPP_WARN(rclcpp::get_logger("NMPCSMCCUBE"), "Received incorrect number of trajectory data.");
            return;
        }
        xr = msg->data[0];
        yr = msg->data[1];
        zr = msg->data[2];
    }

    void sidepropCallback (const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 2) {
            RCLCPP_WARN(rclcpp::get_logger("NMPCSMCCUBE"), "Received incorrect number of side propellers data.");
            return;
        }
        w9 = msg->data[0];
        w10 = msg->data[1];
    }

    void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        
        const auto &position = msg->pose.pose.position;
        const auto &orientation = msg->pose.pose.orientation;
        const auto &linear_velocity = msg->twist.twist.linear;
        const auto &angular_velocity = msg->twist.twist.angular;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 rpy(q);
        double roll, pitch, yaw;
        rpy.getRPY(roll, pitch, yaw);

        double x = position.x;
        double y = position.y;
        double z = position.z;
        double phi = roll; 
        double theta = pitch;  
        double psi = yaw; 
        double xd = linear_velocity.x;
        double yd = linear_velocity.y;
        double zd = linear_velocity.z;
        double phid = angular_velocity.x;
        double thetad = angular_velocity.y;
        double psid = angular_velocity.z;

        rclcpp::Time timenow = this->get_clock()->now();
        double dt = (timenow - pretime_).seconds();
        pretime_ = timenow;
        
        double xrd = 0, yrd = 0, zrd = 0;
        double xrdd = 0, yrdd = 0, zrdd = 0;
        double ixx = 0.0785, iyy = 0.0785, izz = 0.105;

        auto [phir, thetar] = controlxy_.calculatexy(xr, yr, x, y, phi, theta, psi, dt);
        double phird = 0;
        double thetard = 0;
        double psird = 0;

        double cphi_ctrl = 12.5;
        double ctheta_ctrl = 12.5;
        double cpsi_ctrl = 12.5;
        
        double Kx = 32.5;
        double Ky = 32.5;
        double Kz = 27.5;

        double fphi = psid * thetad * (iyy - izz) / ixx;
        double ftheta = psid * phid * (izz - ixx) / iyy;
        double fpsi = phid * thetad * (ixx - iyy) / izz;

        double bphi = 1 / ixx;
        double btheta = 1 / iyy;
        double bpsi = 1 / izz;

        double Kphi = 1.75;
        double Ktheta = 1.75;
        double Kpsi = 0.75;

        double scphi = cphi_ctrl * (phi - phir) + (phid - phird);
        double sctheta = ctheta_ctrl * (theta - thetar) + (thetad - thetard);
        double scpsi = cpsi_ctrl * (psi - psir) + (psid - psird);

        double satphi = clamp(scphi, -0.1, 0.1);
        double sattheta = clamp(sctheta, -0.1, 0.1);
        double satpsi = clamp(scpsi, -0.1, 0.1);

        double U2s = (-Ky * cphi_ctrl * phi - (cphi_ctrl + Ky) * phid + Ky * cphi_ctrl * phir + (cphi_ctrl + Ky) * phird - fphi - Kphi * satphi) / bphi;
        double U3s = (-Kx * ctheta_ctrl * theta - (ctheta_ctrl + Kx) * thetad + Kx * ctheta_ctrl * thetar + (ctheta_ctrl + Kx) * thetard - ftheta - Ktheta * sattheta) / btheta;
        double U4s = (-Kz * cpsi_ctrl * psi - (cpsi_ctrl + Kz) * psid + Kz * cpsi_ctrl * psir + (cpsi_ctrl + Kz) * psird - fpsi - Kpsi * satpsi) / bpsi;

        double U2smc = clamp(U2s, -3.75, 3.75);
        double U3smc = clamp(U3s, -3.75, 3.75);
        double U4smc = clamp(U4s, -3.75, 3.75);

        double m = 5.0;
        double g = 9.80;
        double Kdx = 0.0000267;
        double Kdy = 0.0000267;
        double Kdz = 0.0000625;
        double cx = 0.015;
        double cy = 0.015;
        double cz = 0.375;
        
        double lam1 = 0.05;
        double lam2 = 0.05;
        double Ka = 10.75;
        double eta = 0.25;

        double cphi = cos(phi), sphi = sin(phi);
        double ctheta = cos(theta), stheta = sin(theta);
        double cpsi = cos(psi), spsi = sin(psi);

        double fx = -Kdx * xd / m;
        double fy = -Kdy * yd / m;
        double fz = (-Kdz * zd - m * g) / m;
        double bx = 1 / m * (spsi * sphi + cpsi * stheta * cphi);
        double by = 1 / m * (spsi * stheta * cphi - cpsi * sphi);
        double bz = 1 / m * (ctheta * cphi);

        double sx = cx * (xr - x) + (xrd - xd);
        double sy = cy * (yr - y) + (yrd - yd);
        double sz = cz * (zr - z) + (zrd - zd);

        
        double ueqx = (cx * (xrd - xd) + xrdd - fx) / bx;
        double ueqy = (cy * (yrd - yd) + yrdd - fy) / by;
        double ueqz = (cz * (zrd - zd) + zrdd - fz) / bz;

        double s3 = lam2 * lam1 * sx + lam2 * sy + sz;
        double sats3 = clamp(s3, -0.1, 0.1);

        double usw = -(lam2 * lam1 * bx * (ueqy + ueqz) + lam2 * by * (ueqx + ueqz) + bz * (ueqx + ueqy) - Ka * s3 - eta * sats3) / (lam2 * lam1 * bx + lam2 * by + bz);
        double Uz = ueqx + ueqy + ueqz + usw;

        double U1smc = clamp(Uz, 40.0, 60.0);

        unsigned int i, iter;
        acado_timer t;

        std::memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
        std::memset(&acadoVariables, 0, sizeof(acadoVariables));

        acado_initializeSolver();

        for (i = 0; i < N + 1; ++i)
        {
            acadoVariables.x[i * NX + 0] = x;
            acadoVariables.x[i * NX + 1] = y;
            acadoVariables.x[i * NX + 2] = z;
            acadoVariables.x[i * NX + 3] = phi;
            acadoVariables.x[i * NX + 4] = theta;
            acadoVariables.x[i * NX + 5] = psi;
            acadoVariables.x[i * NX + 6] = xd;
            acadoVariables.x[i * NX + 7] = yd;
            acadoVariables.x[i * NX + 8] = zd;
            acadoVariables.x[i * NX + 9] = phid;
            acadoVariables.x[i * NX + 10] = thetad;
            acadoVariables.x[i * NX + 11] = psid;
        }
    
        for (i = 0; i < N; ++i) {
            acadoVariables.y[i * NY + 0] = zr; 
            acadoVariables.y[i * NY + 1] = phir;   
            acadoVariables.y[i * NY + 2] = thetar;   
            acadoVariables.y[i * NY + 3] = psir;
            acadoVariables.y[i * NY + 4] = 0; 
            acadoVariables.y[i * NY + 5] = U1smc;  
            acadoVariables.y[i * NY + 6] = U2smc;
            acadoVariables.y[i * NY + 7] = U3smc;   
            acadoVariables.y[i * NY + 8] = U4smc;  
        }

        acadoVariables.yN[0] = zr;  
        acadoVariables.yN[1] = phir;   
        acadoVariables.yN[2] = thetar;   
        acadoVariables.yN[3] = psir;
        for (i = 0; i < NX; ++i) {   
            acadoVariables.x0[ i ] = acadoVariables.x[NX + i];
        }

        acado_preparationStep();
        acado_feedbackStep();
        acado_shiftStates(2, 0, 0);
        acado_shiftControls(0);
        real_t* u = acado_getVariablesU();

        std::vector<std::vector<double>> control_variables;
        for (int i = 0; i < N; ++i) { 
            std::vector<double> control_row;
            for (int j = 0; j < NU; ++j) { 
                control_row.push_back(static_cast<double>(u[i * NU + j]));
            }
            control_variables.push_back(control_row);
        }
        //SM_NMPC
        double Fy = control_variables[0][0];
        double U1 = control_variables[0][1];
        double U2 = control_variables[0][2];
        double U3 = control_variables[0][3];
        double U4 = control_variables[0][4];

        //SMC
        //double U1 = U1smc;
        //double U2 = U2smc;
        //double U3 = U3smc;
        //double U4 = U4smc;
        
        double kt = 0.00025, kd = 0.000075, ly = 0.1845, lx= 0.219;
        double w12, w22, w32, w42;

        w12 = (U1*kd*lx*ly - U2*kd*lx - U3*kd*ly + U4*kt*lx*ly) / (8 * kd * kt * lx * ly);
        w22 = (U1*kd*lx*ly + U2*kd*lx - U3*kd*ly - U4*kt*lx*ly) / (8 * kd * kt * lx * ly);
        w32 = (U1*kd*lx*ly - U2*kd*lx + U3*kd*ly - U4*kt*lx*ly) / (8 * kd * kt * lx * ly);
        w42 = (U1*kd*lx*ly + U2*kd*lx + U3*kd*ly + U4*kt*lx*ly) / (8 * kd * kt * lx * ly);

        w12 = std::max(0.0, w12);
        w22 = std::max(0.0, w22);
        w32 = std::max(0.0, w32);
        w42 = std::max(0.0, w42);

        double w1 = -std::sqrt(w12);
        double w2 = std::sqrt(w22);
        double w3 = std::sqrt(w32);
        double w4 = -std::sqrt(w42);

        std_msgs::msg::Float64MultiArray prop_vel_msg;
        prop_vel_msg.data = {w1, w2, w3, w4, w1, w2, w3, w4, w9, w10};
        propvel_pub_->publish(prop_vel_msg);

        //auto node = rclcpp::Node::make_shared("time_logger");
        //rclcpp::Clock::SharedPtr clock = node->get_clock();
        //auto now = clock->now();
        //auto elapsed_time = now.nanoseconds(); // ROS time in nanoseconds
        //auto elapsed_time_ms = elapsed_time / 1'000'000; 
        //std::ofstream outfile1("datasmcnmpccubefailure.txt", std::ios_base::app);
        //if (outfile1.is_open()) {
        //    outfile1 << "Time: " << elapsed_time_ms << ", x: " << x << ", y: " << y << ", z: " << z << ", xd: " << xd << ", yd: " << yd << ", zd: " << zd << ", phi: " << phi << ", theta: " << theta << ", psi: " << psi << ", U1: " << U1 << ", U2: " << U2 << ", U3: " << U3 << ", U4: " << U4 << "\n";
        //   outfile1.close();
        //} else {
        //    RCLCPP_ERROR(node->get_logger(), "Unable to open file for writing.");
        //}
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr propvel_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr trajectory_sub_; 
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sideprop_sub_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;

    rclcpp::Time pretime_;
    double x, y, z, xd, yd, zd, phi, theta, psi, phid, thetad, psid;
    double xr, yr, zr, psir, w9, w10, addforce;
    PIDXY controlxy_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NMPCSMCCUBE>());
    rclcpp::shutdown();
    return 0;
}
