// mandatory includes for base code
#include <rclcpp/rclcpp.hpp>
#include <ecn_2024/tracker.h>
#include <ecn_2024/srv/letter.hpp>
#include <nav_msgs/msg/path.hpp>

// include any thing required - do not forget to use the .hpp extension for ROS 2 files



using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control")
  {
    // the path will be in the map frame
    path.header.frame_id = "map";


    // TODO init the frame of the robot from a parameter
    // tracker.setFrame(some_frame);

    // TODO init publishers and timer

    // TODO create a Letter request and fill it with parameters

    // TODO call the service and give a callback function for the result


    // init member variables such as publishers / subscribers / timers


  }
  
private:
  // helper for the control law
  Tracker tracker{this};

  // (optional) the path to publish
  nav_msgs::msg::Path path;

  // TODO declare member variables such as publishers / subscribers / timers



  // callback for the service, to be filled
  void client_cb(rclcpp::Client<ecn_2024::srv::Letter>::SharedFuture future)
  {
    auto result{future.get()};
    if(!result)
      return;

    // TODO init the tracker from the result (it is a pointer) and fill the path message


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
