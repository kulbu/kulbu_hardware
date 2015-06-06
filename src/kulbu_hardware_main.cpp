/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#include <kulbu_hardware/kulbu_hardware_control_loop.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "kulbu_hardware_interface");
  ros::NodeHandle nh;

  // NOTE: We run the ROS loop in a separate thread as external calls such
  // as service callbacks to load controllers can block the (main) control loop
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Create the hardware interface specific to your robot
  int joint_mode = 1;  // position
  boost::shared_ptr<kulbu_hardware::KulbuHardwareInterface> hardware_interface;
  hardware_interface.reset(
    new kulbu_hardware::KulbuHardwareInterface(nh, joint_mode));

  // Start the control loop
  kulbu_hardware::KulbuHardwareControlLoop control_loop(nh, hardware_interface);

  // Wait until shutdown signal recieved
  ros::waitForShutdown();

  return 0;
}
