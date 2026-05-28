// mandatory includes for base code
#include <rclcpp/rclcpp.hpp>
#include <ecn_2025/srv_sync.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <baxter_simple_sim/srv/jacobian.hpp>

// TODO include any missing header - do not forget to use the .hpp extension for ROS 2 files


/// helper function to compute the control law
std::vector<double> computeVisualServo(const geometry_msgs::msg::Transform &cMo,
                                       const std::array<double, 42> &eJe);


/// helper function to build the joint names for a given arm
auto buildJointNames(const std::string &side)
{
  std::vector<std::string> names;
  names.reserve(7);
  for(auto suffix: {"s0", "s1", "e0", "e1", "w0", "w1", "w2"})
    names.push_back(side + "_" + suffix);
  return names;
}


using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control")
  {
    // TODO init cmd message with joint names and command mode (velocity)

    // TODO create command publisher

    // TODO init jacobian client, you can also prepare the request

    // timer to run the control loop
    timer = create_wall_timer(100ms, [this](){controlLoop();});
  }
  
private:

  // TODO change to a parameter later on to make the node arm-agnostic
  std::string side{"left"};

  // TF 2 stuff  
  tf2_ros::Buffer buffer{get_clock()};
  tf2_ros::TransformListener listener{buffer};

  // Jacobian service
  ServiceNodeSync<baxter_simple_sim::srv::Jacobian> jac_client;

  // TODO arm control



  // timer
  rclcpp::TimerBase::SharedPtr timer;

  void controlLoop()
  {
    // TODO check the transform is feasible, or return

    // TODO get transform

    // TODO get jacobian

    // TODO compute visual servo and publish command
  }
};



// boilerplate main function
int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
