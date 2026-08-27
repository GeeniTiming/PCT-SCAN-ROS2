#include <functional>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {
double foxyTf2ApiSmoke() {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, 0.25);
  const auto orientation = tf2::toMsg(quaternion);
  return tf2::getYaw(orientation);
}
}  // namespace

class FoxySubscriptionSmoke : public rclcpp::Node {
 public:
  FoxySubscriptionSmoke() : Node("foxy_subscription_smoke") {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&FoxySubscriptionSmoke::odomCallback, this,
                  std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud", rclcpp::SensorDataQoS(),
        std::bind(&FoxySubscriptionSmoke::cloudCallback, this,
                  std::placeholders::_1));
  }

 private:
  void odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr) {}
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr) {}

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FoxySubscriptionSmoke>();
  rclcpp::shutdown();
  return node && foxyTf2ApiSmoke() > 0.0 ? 0 : 1;
}
