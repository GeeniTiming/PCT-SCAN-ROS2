#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "bspline_opt/uniform_bspline.h"
#include "plan_env/pipeline_trace.h"

namespace scan_planner
{
class ClosedLoopController : public rclcpp::Node
{
public:
  ClosedLoopController() : Node("closed_loop_controller")
  {
    time_forward_ = declare_parameter<double>("time_forward", 0.8);
    heading_error_threshold_ = declare_parameter<double>("heading_error_threshold", 0.8);
    kp_pos_ = declare_parameter<double>("kp_pos", 0.8);
    kp_yaw_ = declare_parameter<double>("kp_yaw", 1.5);
    kd_pos_ = declare_parameter<double>("kd_pos", 0.0);
    kd_yaw_ = declare_parameter<double>("kd_yaw", 0.0);
    max_vx_ = declare_parameter<double>("max_vx", 0.75);
    max_vy_ = declare_parameter<double>("max_vy", 0.35);
    max_vyaw_ = std::min(declare_parameter<double>("max_vyaw", 1.0), kMaxVYawLimit);
    finish_dist_ = declare_parameter<double>("finish_dist", 0.15);
    finish_yaw_tolerance_ = declare_parameter<double>("finish_yaw_tolerance", 0.08);
    const std::string bspline_topic = declare_parameter<std::string>("bspline_topic", "planning/bspline");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "body_pose");
    const std::string cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    const std::string frozen_topic = declare_parameter<std::string>("frozen_topic", "planning/go2_execution_frozen");

    bspline_sub_ = create_subscription<scan_planner_msgs::msg::Bspline>(
        bspline_topic, 10,
        std::bind(&ClosedLoopController::bsplineCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        std::bind(&ClosedLoopController::odomCallback, this, std::placeholders::_1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 20);
    execution_frozen_pub_ = create_publisher<std_msgs::msg::Bool>(frozen_topic, 10);
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&ClosedLoopController::cmdCallback, this));
    RCLCPP_INFO(get_logger(), "Closed-loop controller ready");
  }

private:
  static constexpr double kMaxVYawLimit = 1.0;

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
  {
    const double norm = value.norm();
    return (norm <= max_norm || norm < 1e-6) ? value : value / norm * max_norm;
  }

  // Sample the planner-supplied yaw profile (equal-interval samples plus linear
  // interpolation) at trajectory time t.  Only used when has_yaw_profile_.
  double sampleYaw(double t) const
  {
    const double tc = std::clamp(t, 0.0, traj_duration_);
    const double u = tc / std::max(yaw_dt_, 1e-6);
    const int n = static_cast<int>(yaw_pts_.size());
    if (n == 0) return odom_yaw_;
    const int i0 = std::min<int>(static_cast<int>(u), n - 1);
    const int i1 = std::min<int>(i0 + 1, n - 1);
    const double frac = std::min(u - i0, 1.0);
    return yaw_pts_[i0] + normalizeAngle(yaw_pts_[i1] - yaw_pts_[i0]) * frac;
  }

  double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des) const
  {
    if (has_yaw_profile_) return sampleYaw(t_cur);
    const double t_look = std::min(traj_duration_, t_cur + time_forward_);
    Eigen::Vector3d direction = traj_[0].evaluateDeBoorT(t_look) - pos_des;
    if (direction.head<2>().squaredNorm() < 1e-4)
      direction = traj_[1].evaluateDeBoorT(t_cur);
    return direction.head<2>().squaredNorm() < 1e-4
        ? odom_yaw_ : std::atan2(direction.y(), direction.x());
  }

  void publishCommand(const geometry_msgs::msg::Twist& command) {
    cmd_vel_pub_->publish(command);
    ++trace_cmd_seq_;
    if (plan_env::trace::enabled()) plan_env::trace::event("command_publish",
        ",\"cmd_seq\":" + std::to_string(trace_cmd_seq_) +
        ",\"traj_id\":" + std::to_string(traj_id_) +
        ",\"control_begin_ns\":" + std::to_string(trace_control_begin_ns_) +
        ",\"odom_stamp_ns\":" + std::to_string(trace_odom_stamp_ns_) +
        ",\"vx\":" + std::to_string(command.linear.x) +
        ",\"vy\":" + std::to_string(command.linear.y) +
        ",\"wz\":" + std::to_string(command.angular.z));
  }

  void publishStop(double yaw_rate = 0.0)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(yaw_rate, -max_vyaw_, max_vyaw_);
    publishCommand(cmd);
  }

  void publishExecutionFrozen(bool frozen)
  {
    std_msgs::msg::Bool msg;
    msg.data = frozen;
    execution_frozen_pub_->publish(msg);
  }

  void bsplineCallback(const scan_planner_msgs::msg::Bspline::ConstSharedPtr msg)
  {
    if (msg->pos_pts.empty() || msg->knots.empty() || msg->order <= 0)
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid B-spline");
      return;
    }
    Eigen::MatrixXd points(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
      points.col(i) << msg->pos_pts[i].x, msg->pos_pts[i].y, msg->pos_pts[i].z;
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i) knots(i) = msg->knots[i];
    UniformBspline position(points, msg->order, 0.1);
    position.setKnot(knots);
    traj_ = {position, position.getDerivative()};
    traj_.push_back(traj_[1].getDerivative());
    traj_duration_ = traj_[0].getTimeSum();
    yaw_pts_ = msg->yaw_pts;
    yaw_dt_ = msg->yaw_dt;
    has_yaw_profile_ = !yaw_pts_.empty() && yaw_dt_ > 0.0;
    terminal_hold_ = has_yaw_profile_ && points.cols() > 0;
    for (Eigen::Index i = 1; i < points.cols(); ++i)
      terminal_hold_ = terminal_hold_ && (points.col(i) - points.col(0)).norm() < 1e-6;
    traj_id_ = msg->traj_id;
    if (plan_env::trace::enabled()) plan_env::trace::event("trajectory_received",
        ",\"traj_id\":" + std::to_string(traj_id_));
    // 直接采用消息里的 start_time 作为轨迹时间原点：规划器在 freeze 时会前推
    // start_time，因此这里天然与规划器时钟一致，重规划也不会把参考点强制拉回 0。
    if (msg->start_time.sec == 0 && msg->start_time.nanosec == 0)
      traj_start_time_ = now();
    else
      traj_start_time_ = rclcpp::Time(msg->start_time, get_clock()->get_clock_type());
    receive_traj_ = true;
    RCLCPP_INFO(get_logger(), "Received trajectory %lld, duration %.3fs",
                static_cast<long long>(traj_id_), traj_duration_);
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    trace_odom_stamp_ns_ = static_cast<long long>(msg->header.stamp.sec)*1000000000LL+msg->header.stamp.nanosec;
    odom_pos_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    // /lightning/odom 的 twist 是 map 系速度（loc_system.cc: velocity = pose.so3() * vel_b_）
    odom_vel_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    const double yaw = tf2::getYaw(msg->pose.pose.orientation);
    const double stamp_sec = now().seconds();
    if (have_odom_)
    {
      const double dt = stamp_sec - last_odom_stamp_sec_;
      if (dt > 1e-3)
        odom_yaw_rate_ = normalizeAngle(yaw - odom_yaw_) / dt;
    }
    odom_yaw_ = yaw;
    last_odom_stamp_sec_ = stamp_sec;
    have_odom_ = true;
  }

  void cmdCallback()
  {
    trace_control_begin_ns_ = plan_env::trace::enabled() ? plan_env::trace::steadyNs() : 0;
    if (!receive_traj_ || !have_odom_)
    {
      publishExecutionFrozen(false);
      publishStop();
      return;
    }
    // 参考时间由 traj_start_time_ 推算，不再每条消息清零/按墙上时钟累加。
    const double raw_t = (now() - traj_start_time_).seconds();
    const double t_eval = std::clamp(raw_t, 0.0, traj_duration_);
    Eigen::Vector3d pos_des = traj_[0].evaluateDeBoorT(t_eval);
    const double yaw_error = normalizeAngle(estimateDesiredYaw(t_eval, pos_des) - odom_yaw_);
    const double yaw_command =
        std::clamp(kp_yaw_ * yaw_error - kd_yaw_ * odom_yaw_rate_, -max_vyaw_, max_vyaw_);
    if (terminal_hold_)
    {
      publishExecutionFrozen(false);
      // Finish turning without feeding odometry noise into translation.
      publishStop(std::abs(yaw_error) > finish_yaw_tolerance_ ? yaw_command : 0.0);
      return;
    }
    // A yaw profile deliberately turns the robot while it moves, so the
    // heading-error freeze must not stop it; it only applies to tangent control.
    if (!has_yaw_profile_ && std::abs(yaw_error) > heading_error_threshold_)
    {
      publishExecutionFrozen(true);
      publishStop(yaw_command);
      return;
    }

    publishExecutionFrozen(false);
    const Eigen::Vector3d vel_des = traj_[1].evaluateDeBoorT(t_eval);
    const Eigen::Vector2d pos_error(pos_des.x() - odom_pos_.x(), pos_des.y() - odom_pos_.y());
    const Eigen::Vector2d vel_des_xy(vel_des.x(), vel_des.y());
    const Eigen::Vector2d odom_vel_xy(odom_vel_.x(), odom_vel_.y());
    // PD 跟踪：vel_des 前馈 + 位置比例 + 速度阻尼。纯 P 会过冲并触发反向命令
    // （即“走一截回退一小截”），阻尼项把过冲压掉。
    const Eigen::Vector2d vel_world = clampNorm(
        vel_des_xy + kp_pos_ * pos_error - kd_pos_ * (odom_vel_xy - vel_des_xy),
        std::max(max_vx_, max_vy_));
    const double c = std::cos(odom_yaw_);
    const double s = std::sin(odom_yaw_);
    geometry_msgs::msg::Twist command;
    // 该场景沿 PCT 参考路径单向前进，不需要倒车。轨迹末端 vel_des≈0 时，
    // 位置保持项 + 2.5Hz 滞后 odom 会过冲成反向指令（即“走一步回退”），
    // 因此把纵向速度钳成非负，末端只会停住/微前蠕，不再倒退。
    command.linear.x = std::clamp(c * vel_world.x() + s * vel_world.y(), 0.0, max_vx_);
    command.linear.y = std::clamp(-s * vel_world.x() + c * vel_world.y(), -max_vy_, max_vy_);
    command.angular.z = yaw_command;
    if (raw_t >= traj_duration_ && pos_error.norm() < finish_dist_)
    {
      if (has_yaw_profile_ && std::abs(yaw_error) > finish_yaw_tolerance_)
      {
        // Position is reached but the terminal heading is not: rotate in place
        // instead of zeroing everything and stopping mis-aimed.
        geometry_msgs::msg::Twist rotate_cmd;
        rotate_cmd.angular.z = yaw_command;
        publishCommand(rotate_cmd);
        return;
      }
      command = geometry_msgs::msg::Twist();
    }
    publishCommand(command);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_frozen_pub_;
  rclcpp::Subscription<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
  std::uint64_t trace_cmd_seq_{0};
  long long trace_control_begin_ns_{0}, trace_odom_stamp_ns_{0};
  bool receive_traj_{false};
  bool have_odom_{false};
  std::vector<UniformBspline> traj_;
  double traj_duration_{0.0};
  std::int64_t traj_id_{0};
  rclcpp::Time traj_start_time_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d odom_vel_{Eigen::Vector3d::Zero()};
  double odom_yaw_{0.0};
  double odom_yaw_rate_{0.0};
  double last_odom_stamp_sec_{-1.0};
  double time_forward_, heading_error_threshold_, kp_pos_, kp_yaw_;
  double kd_pos_{0.0}, kd_yaw_{0.0};
  double max_vx_, max_vy_, max_vyaw_, finish_dist_, finish_yaw_tolerance_;
  std::vector<double> yaw_pts_;
  double yaw_dt_{0.0};
  bool has_yaw_profile_{false};
  bool terminal_hold_{false};
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::ClosedLoopController>());
  rclcpp::shutdown();
  return 0;
}
