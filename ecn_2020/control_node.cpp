// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;


namespace ecn_2020
{

class ControlNode : public rclcpp::Node
{
public:
  ControlNode(rclcpp::NodeOptions options)
      : Node("control", options), tf_buffer(get_clock()), tf_listener(tf_buffer)
  {
    // init parameters: name of this robot + name of the target

    base_frame = declare_parameter<std::string>("base_frame", "bb8/base_link");

    target_frame = declare_parameter<std::string>("target_frame", "r2d2/base_link");





    // init publishers / subscribers / timers
    cmd_vel_pub = this->create_publisher<Twist>("cmd_vel", 10);
    timer = this->create_wall_timer(100ms, std::bind(&ControlNode::update, this));

    joint_state_pub = this->create_publisher<JointState>("joint_states", 10);
    joint_state_msg.name = {"wheel", "torso", "neck"};
    joint_state_msg.position = {0.0, 0.0, 0.0};


  }
  
private:

  // declare member variables for publisher and timer
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr timer;

  rclcpp::Publisher<JointState>::SharedPtr joint_state_pub;
  JointState joint_state_msg;

  // base and target frames
  std::string base_frame{"bb8/base_link"};
  std::string target_frame{"r2d2/base_link"};

  // TF 2 stuff
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void update()
  {

    if(not tf_buffer.canTransform(target_frame, base_frame, rclcpp::Time()))
      return;

    const auto t{tf_buffer.lookupTransform(base_frame, target_frame, rclcpp::Time()).transform.translation};

    Twist cmd;
    cmd.linear.x = 2*(t.x -1);
    cmd.angular.z = 2*atan2(t.y, t.x);

    cmd_vel_pub->publish(cmd);


    // update joint states
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.position[0] += 3.7*cmd.linear.x*0.1;
    joint_state_msg.position[1] = 0.5*cmd.linear.x;
    joint_state_msg.position[2] = 0.5*cmd.angular.z;
    joint_state_pub->publish(joint_state_msg);




  }

  
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ecn_2020::ControlNode)
