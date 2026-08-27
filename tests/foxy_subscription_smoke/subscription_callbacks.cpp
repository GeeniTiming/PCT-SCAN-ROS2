#include <functional>
#include <memory>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

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
  return node ? 0 : 1;
}
