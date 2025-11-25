#ifndef TRACKER_H_
#define TRACKER_H_

#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>

inline auto toPi(double angle)
{
  while(angle > M_PI)
    angle -= 2*M_PI;
  while(angle < -M_PI)
    angle += 2*M_PI;
  return angle;
}

class Tracker
{
  using Point = geometry_msgs::msg::Point;
public:
  Tracker(rclcpp::Node* node) : buffer{node->get_clock()}, listener{buffer}
  {
  }

  void setFrame(const std::string& frame)
  {
    this->frame = frame;
  }

  void initFrom(const std::vector<geometry_msgs::msg::Point>& contour)
  {

    assert(contour.size() > 0);
    this->contour = contour;
    cur = this->contour.begin();
  }

  std::pair<double,double> track()
  {
    if(contour.empty() || !buffer.canTransform(frame, "map", tf2::TimePointZero))
      return {};

    const auto me{buffer.lookupTransform("map", frame, tf2::TimePointZero).transform};

    const auto [dx,dy] = updateWP(me.translation.x, me.translation.y);
    const auto theta{2*atan2(me.rotation.z, me.rotation.w)};
    const auto c{cos(theta)};
    const auto s{sin(theta)};
    const auto dtheta{toPi(atan2(dy,dx) - theta)};

    return {std::clamp(Kv*(dx*c+dy*s), -vmax, vmax) * (M_PI-std::abs(dtheta)),
            Kw * dtheta};
  }

private:
  std::string frame;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
  std::vector<Point> contour;
  std::vector<Point>::iterator cur;

  constexpr static auto thr{0.1};
  constexpr static auto vmax{2.};
  constexpr static auto Kv{3.};
  constexpr static auto Kw{6.};

  std::pair<double,double> updateWP(double x, double y)
  {
    const auto dx{cur->x - x};
    const auto dy{cur->y - y};

    if(sqrt(dx*dx+dy*dy) > thr)
      return {dx,dy};

    cur++;
    if(cur == contour.end())
      cur = contour.begin();
    return {cur->x - x, cur->y - y};
  }
};

#endif // TRACKER_H_
