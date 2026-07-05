#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

namespace
{

struct Point2
{
  double x = 0.0;
  double y = 0.0;
};

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double distance(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point2 transform_to_base(double world_x, double world_y, double robot_x, double robot_y, double robot_yaw)
{
  const double dx = world_x - robot_x;
  const double dy = world_y - robot_y;
  return {
    std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy,
    -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy};
}

}  // namespace

class ObstacleAvoidPurePursuit : public rclcpp::Node
{
public:
  ObstacleAvoidPurePursuit()
  : Node("obstacle_avoid_pure_pursuit")
  {
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    path_topic_ = declare_parameter("path_topic", "/path");
    scan_topic_ = declare_parameter("scan_topic", "/scan");
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", "/cmd_vel");

    lookahead_distance_ = declare_parameter("lookahead_distance", 0.4);
    target_speed_ = declare_parameter("target_speed", 0.15);
    max_angular_speed_ = declare_parameter("max_angular_speed", 1.2);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.08);
    robot_radius_ = declare_parameter("robot_radius", 0.18);
    safety_margin_ = declare_parameter("safety_margin", 0.07);

    front_check_distance_ = declare_parameter("front_check_distance", 0.6);
    front_check_angle_deg_ = declare_parameter("front_check_angle_deg", 120.0);
    obstacle_consider_distance_ = declare_parameter("obstacle_consider_distance", 1.2);
    avoidance_radius_ = declare_parameter("avoidance_radius", 0.45);
    line_check_step_ = declare_parameter("line_check_step", 0.05);
    prediction_time_ = declare_parameter("prediction_time", 1.2);
    prediction_dt_ = declare_parameter("prediction_dt", 0.05);
    odom_timeout_sec_ = declare_parameter("odom_timeout_sec", 0.5);
    scan_timeout_sec_ = declare_parameter("scan_timeout_sec", 0.5);
    const double controller_rate_hz = declare_parameter("controller_rate_hz", 20.0);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidPurePursuit::on_odom, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&ObstacleAvoidPurePursuit::on_path, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObstacleAvoidPurePursuit::on_scan, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, controller_rate_hz));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ObstacleAvoidPurePursuit::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "Obstacle avoidance Pure Pursuit: odom=%s path=%s scan=%s cmd_vel=%s",
      odom_topic_.c_str(), path_topic_.c_str(), scan_topic_.c_str(), cmd_vel_topic_.c_str());
  }

  ~ObstacleAvoidPurePursuit() override
  {
    publish_stop();
  }

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_yaw_ = yaw_from_quaternion(msg->pose.pose.orientation);
    has_odom_ = true;
    last_odom_time_ = now();
  }

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_points_.clear();
    nearest_front_obstacle_.reset();

    const double half_front_angle = front_check_angle_deg_ * M_PI / 360.0;
    double nearest_front_range = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double range = msg->ranges[i];
      if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }
      if (range > obstacle_consider_distance_) {
        continue;
      }

      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      Point2 p{range * std::cos(angle), range * std::sin(angle)};
      scan_points_.push_back(p);

      if (std::abs(angle) <= half_front_angle && range <= front_check_distance_ &&
        range < nearest_front_range)
      {
        nearest_front_range = range;
        nearest_front_obstacle_ = p;
      }
    }

    has_scan_ = true;
    last_scan_time_ = now();
  }

  void on_timer()
  {
    if (!inputs_ready()) {
      publish_stop();
      return;
    }

    if (goal_reached()) {
      publish_stop();
      return;
    }

    const auto ref_target = get_lookahead_point();
    if (!ref_target) {
      publish_stop();
      return;
    }

    Point2 target = *ref_target;
    if (nearest_front_obstacle_) {
      const auto candidates = get_avoidance_candidates(*nearest_front_obstacle_);
      const auto avoid_target = select_safe_target(candidates, *ref_target);
      if (!avoid_target) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "No safe avoidance lookahead candidate. Stopping.");
        publish_stop();
        return;
      }
      target = *avoid_target;
    }

    auto cmd = pure_pursuit_command(target);
    if (trajectory_collision_check(cmd)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Predicted command collides with obstacle data. Stopping.");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
    last_target_ = target;
  }

  bool inputs_ready() const
  {
    const bool odom_fresh = has_odom_ && (now() - last_odom_time_).seconds() <= odom_timeout_sec_;
    const bool scan_fresh = has_scan_ && (now() - last_scan_time_).seconds() <= scan_timeout_sec_;
    return odom_fresh && scan_fresh && has_path_;
  }

  bool goal_reached() const
  {
    if (path_.poses.empty()) {
      return false;
    }
    const auto & goal = path_.poses.back().pose.position;
    return std::hypot(goal.x - robot_x_, goal.y - robot_y_) <= goal_tolerance_;
  }

  std::optional<Point2> get_lookahead_point() const
  {
    if (path_.poses.empty()) {
      return std::nullopt;
    }

    size_t closest_index = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < path_.poses.size(); ++i) {
      const auto & p = path_.poses[i].pose.position;
      const double d = std::hypot(p.x - robot_x_, p.y - robot_y_);
      if (d < closest_dist) {
        closest_dist = d;
        closest_index = i;
      }
    }

    for (size_t i = closest_index; i < path_.poses.size(); ++i) {
      const auto & p = path_.poses[i].pose.position;
      const Point2 base = transform_to_base(p.x, p.y, robot_x_, robot_y_, robot_yaw_);
      if (base.x > 0.0 && std::hypot(base.x, base.y) >= lookahead_distance_) {
        return base;
      }
    }

    const auto & last = path_.poses.back().pose.position;
    return transform_to_base(last.x, last.y, robot_x_, robot_y_, robot_yaw_);
  }

  std::vector<Point2> get_avoidance_candidates(const Point2 & obstacle) const
  {
    std::vector<Point2> candidates;
    const double d = std::hypot(obstacle.x, obstacle.y);

    if (d > 1e-6 && d <= lookahead_distance_ + avoidance_radius_ &&
      d >= std::abs(lookahead_distance_ - avoidance_radius_))
    {
      const double a =
        (lookahead_distance_ * lookahead_distance_ - avoidance_radius_ * avoidance_radius_ + d * d) /
        (2.0 * d);
      const double h_sq = lookahead_distance_ * lookahead_distance_ - a * a;
      if (h_sq >= 0.0) {
        const double h = std::sqrt(h_sq);
        const double ux = obstacle.x / d;
        const double uy = obstacle.y / d;
        const Point2 foot{a * ux, a * uy};
        candidates.push_back({foot.x - h * uy, foot.y + h * ux});
        candidates.push_back({foot.x + h * uy, foot.y - h * ux});
      }
    }

    if (candidates.empty()) {
      const double side = obstacle.y >= 0.0 ? -1.0 : 1.0;
      const double angle = side * 70.0 * M_PI / 180.0;
      candidates.push_back({lookahead_distance_ * std::cos(angle), lookahead_distance_ * std::sin(angle)});
    }

    return candidates;
  }

  std::optional<Point2> select_safe_target(
    const std::vector<Point2> & candidates,
    const Point2 & ref_target) const
  {
    std::optional<Point2> best;
    double best_score = std::numeric_limits<double>::infinity();

    for (const auto & candidate : candidates) {
      if (candidate.x <= 0.03 || collision_check_to_target(candidate)) {
        continue;
      }

      double score = distance(candidate, ref_target);
      if (last_target_) {
        score += 0.25 * distance(candidate, *last_target_);
      }

      if (score < best_score) {
        best_score = score;
        best = candidate;
      }
    }

    return best;
  }

  bool collision_check_to_target(const Point2 & target) const
  {
    const double target_dist = std::hypot(target.x, target.y);
    const int steps = std::max(1, static_cast<int>(std::ceil(target_dist / line_check_step_)));
    const double clearance = robot_radius_ + safety_margin_;

    for (int i = 1; i <= steps; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
      const Point2 sample{target.x * ratio, target.y * ratio};
      for (const auto & obstacle : scan_points_) {
        if (distance(sample, obstacle) <= clearance) {
          return true;
        }
      }
    }

    return false;
  }

  geometry_msgs::msg::Twist pure_pursuit_command(const Point2 & target) const
  {
    geometry_msgs::msg::Twist cmd;
    const double ld = std::max(0.05, std::hypot(target.x, target.y));
    const double curvature = 2.0 * target.y / (ld * ld);
    const double heading_error = std::atan2(target.y, target.x);
    const double speed_scale = clamp(std::cos(heading_error), 0.25, 1.0);

    cmd.linear.x = target_speed_ * speed_scale;
    cmd.angular.z = clamp(target_speed_ * curvature, -max_angular_speed_, max_angular_speed_);
    return cmd;
  }

  bool trajectory_collision_check(const geometry_msgs::msg::Twist & cmd) const
  {
    const double clearance = robot_radius_ + safety_margin_;
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    for (double t = 0.0; t <= prediction_time_; t += prediction_dt_) {
      for (const auto & obstacle : scan_points_) {
        if (std::hypot(obstacle.x - x, obstacle.y - y) <= clearance) {
          return true;
        }
      }

      x += cmd.linear.x * std::cos(yaw) * prediction_dt_;
      y += cmd.linear.x * std::sin(yaw) * prediction_dt_;
      yaw += cmd.angular.z * prediction_dt_;
    }

    return false;
  }

  void publish_stop() const
  {
    if (!cmd_pub_) {
      return;
    }
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
  }

  std::string odom_topic_;
  std::string path_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Path path_;
  std::vector<Point2> scan_points_;
  std::optional<Point2> nearest_front_obstacle_;
  std::optional<Point2> last_target_;

  double lookahead_distance_ = 0.4;
  double target_speed_ = 0.15;
  double max_angular_speed_ = 1.2;
  double goal_tolerance_ = 0.08;
  double robot_radius_ = 0.18;
  double safety_margin_ = 0.07;
  double front_check_distance_ = 0.6;
  double front_check_angle_deg_ = 120.0;
  double obstacle_consider_distance_ = 1.2;
  double avoidance_radius_ = 0.45;
  double line_check_step_ = 0.05;
  double prediction_time_ = 1.2;
  double prediction_dt_ = 0.05;
  double odom_timeout_sec_ = 0.5;
  double scan_timeout_sec_ = 0.5;

  bool has_odom_ = false;
  bool has_path_ = false;
  bool has_scan_ = false;
  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_yaw_ = 0.0;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_scan_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoidPurePursuit>());
  rclcpp::shutdown();
  return 0;
}
