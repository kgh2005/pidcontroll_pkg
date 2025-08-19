#ifndef PIDCONTROLL_PKG__PIDCONTROLL_PKG_HPP_
#define PIDCONTROLL_PKG__PIDCONTROLL_PKG_HPP_

#include <chrono>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

#include "sensor_msgs/msg/imu.hpp"
#include "PID_controller/pidcontroll.h"
#include <intelligent_humanoid_interfaces/msg/vision2_master_msg.hpp>

class PidControllNode : public rclcpp::Node
{
public:
  PidControllNode();

private:

  // Parameters
  double Kp_, Ki_, Kd_;
  int R_YAW_MAX, L_YAW_MAX;

  double yaw_imu = 0.0;

  double line_X = 0.0, line_Y = 0.0;
  double delta = 0.0; 

  PIDControll pid_controller_;

  // ===== ROS 통신 =====
  rclcpp::Subscription<intelligent_humanoid_interfaces::msg::Vision2MasterMsg>::SharedPtr delta_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // ===== Callback =====
  void deltaCallback(const intelligent_humanoid_interfaces::msg::Vision2MasterMsg::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // ===== PID =====
  void pidControll();
};


#endif  // PIDCONTROLL_PKG__PIDCONTROLL_PKG_HPP_
