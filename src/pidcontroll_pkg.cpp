#include "pidcontroll_pkg/pidcontroll_pkg.hpp"

PidControllNode::PidControllNode() : Node("pidcontroll_node"), pid_controller_(0.0, 0.0, 0.0, 1.0, 1.0)
{
  // Declare & get parameters
  this->declare_parameter("Kp", Kp_);
  this->declare_parameter("Ki", Ki_);
  this->declare_parameter("Kd", Kd_);
  this->declare_parameter("R_YAW_MAX", R_YAW_MAX);
  this->declare_parameter("L_YAW_MAX", L_YAW_MAX);

  this->get_parameter("Kp", Kp_);
  this->get_parameter("Ki", Ki_);
  this->get_parameter("Kd", Kd_);
  this->get_parameter("R_YAW_MAX", R_YAW_MAX);
  this->get_parameter("L_YAW_MAX", L_YAW_MAX);

  RCLCPP_INFO(get_logger(),
              "PID params: kp=%f ki=%f kd=%f, R=%d, L=%d", Kp_, Ki_, Kd_, R_YAW_MAX, L_YAW_MAX);

  pid_controller_ = PIDControll(Kp_, Ki_, Kd_, R_YAW_MAX, 1);

  delta_ = this->create_subscription<intelligent_humanoid_interfaces::msg::Vision2MasterMsg>(
      "/vision/data", 10,
      std::bind(&PidControllNode::deltaCallback, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/raw_data", 10,
      std::bind(&PidControllNode::imuCallback, this, std::placeholders::_1));
}

void PidControllNode::pidControll()
{
  int max = 30;  // 타겟의 최대값
  int min = -30; // 타겟의 최소값

  double target_angle = delta;
  target_angle = target_angle > max ? max : target_angle;
  target_angle = target_angle < min ? min : target_angle;

  double reg_err = (target_angle - min) / (max - min) * 2 - 1;
  std::cout << "reg_err : " << reg_err << std::endl;

  pid_controller_.setParams(Kp_, Ki_, Kd_, 1);
  double yaw = pid_controller_.controller(reg_err);
  std::cout << "yaw : " << yaw << std::endl;
  std::cout << "yaw_imu : " << yaw_imu << std::endl;
}

void PidControllNode::deltaCallback(const intelligent_humanoid_interfaces::msg::Vision2MasterMsg::SharedPtr msg)
{
  line_X = msg->line_cam_x;
  line_Y = msg->line_cam_y;

  double tilt_signed_deg = std::atan2(line_X - 400, 550 - line_Y) * 180.0 / M_PI; // 좌: 음수, 우: 양수


  delta = tilt_signed_deg;
}
void PidControllNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double x = msg->orientation.x;
  double y = msg->orientation.y;
  double z = msg->orientation.z;
  double w = msg->orientation.w;

  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (y * y + z * z);
  yaw_imu = std::atan2(t3, t4) * 180.0 / M_PI;

  pidControll();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PidControllNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
