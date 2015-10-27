/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#include <kulbu_hardware/kulbu_hardware_interface.h>

namespace kulbu_hardware {

KulbuHardwareInterface::KulbuHardwareInterface(
  ros::NodeHandle& nh,
  int joint_mode)
  : nh_(nh),
  joint_mode_(joint_mode) {
  // Initialize shared memory and interfaces here
  init();  // this implementation loads from rosparam

  ROS_INFO_NAMED("kulbu_hardware_interface",
    "Loaded kulbu_hardware_interface.");
}

void KulbuHardwareInterface::init() {
  ROS_INFO_STREAM_NAMED("kulbu_hardware_interface",
    "Reading rosparams from namespace: " << nh_.getNamespace());

  nh_.getParam("hardware_interface/steps_per_m", steps_per_m_);
  if (!steps_per_m_ > 0) {
    ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
      "No step per metre found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Get joint names
  nh_.getParam("hardware_interface/joints", joint_names_);
  if (joint_names_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
      "No joints found on parameter server"
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }
  num_joints_ = joint_names_.size();

  // Get direction GPIO pins; 0 = forward, 1 = reverse.
  nh_.getParam("hardware_interface/dirs", pin_dirs_);
  if (pin_dirs_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
      "No GPIO direction pins found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Get PWM sysfs indexes.
  nh_.getParam("hardware_interface/steps", pin_steps_);
  if (pin_steps_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
      "No GPIO step pins found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Pass ros config to robot API.
  robot_api_.pin_dirs  = pin_dirs_;
  robot_api_.pin_steps = pin_steps_;

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
      "Loading joint name: " << joint_names_[i]);

    robot_api_.initJointVelocity(i);

    // Create joint state interface
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
      joint_names_[i],
      &joint_position_[i],
      &joint_velocity_[i],
      &joint_effort_[i]));

    switch (joint_mode_) {
      case 0:
        // Create position joint interface
        position_joint_interface_
          .registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(
              joint_names_[i]),
              &joint_position_command_[i]));
        break;
      case 1:
        // Create velocity joint interface
        velocity_joint_interface_
          .registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(
              joint_names_[i]),
              &joint_velocity_command_[i]));
        break;
      case 2:
        // Create effort joint interface
        effort_joint_interface_
          .registerHandle(hardware_interface::JointHandle(
            joint_state_interface_.getHandle(
              joint_names_[i]),
              &joint_effort_command_[i]));
        break;
    }
  }

  registerInterface(&joint_state_interface_);  // From RobotHW base class.
  registerInterface(&position_joint_interface_);  // From RobotHW base class.
  registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
  registerInterface(&effort_joint_interface_);  // From RobotHW base class.
}

void KulbuHardwareInterface::read(ros::Duration elapsed_time) {
  for (std::size_t i = 0; i < num_joints_; ++i) {
    // Skip joints with no pins defined
    if (pin_dirs_[i] == -1 || pin_steps_[i] == -1) continue;

    switch (joint_mode_) {
      case 1:  // hardware_interface::MODE_VELOCITY:
        joint_velocity_[i] = robot_api_.getJointVelocity(i);

        ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
          "\ni: "     << i <<
          "\nvel: "   << joint_velocity_[i] <<
          "\njoint: " << joint_names_[i]);
        break;

      default:
        ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
          "Joint mode not implemented");
        break;
    }
  }

  // Read the joint states from your hardware here
  // e.g.
  // for (std::size_t i = 0; i < num_joints_; ++i)
  // {
  //   joint_position_[i] = robot_api_.getJointPosition(i);
  //   joint_velocity_[i] = robot_api_.getJointVelocity(i);
  //   joint_effort_[i] = robot_api_.getJointEffort(i);
  // }
}

void KulbuHardwareInterface::write(ros::Duration elapsed_time) {
  unsigned int freq = 0;

  // Send commands in different modes
  for (std::size_t i = 0; i < num_joints_; ++i) {
    // Skip joints with no pins defined
    if (pin_dirs_[i] == -1 || pin_steps_[i] == -1) continue;

    switch (joint_mode_) {
      case 1:  // hardware_interface::MODE_VELOCITY:
        // Calc PWM frequency.
        freq = joint_velocity_command_[i] * steps_per_m_;

        robot_api_.setJointVelocity(i, freq);

       // DEBUG: instantly adding velocity to state
        //joint_velocity_[i] = joint_velocity_command_[i];

        ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
          "\ni: "     << i <<
          "\ncmd: "   << joint_velocity_command_[i] <<
          "\ndist: "  << steps_per_m_ <<
          "\nfreq: "  << freq <<
          "\njoint: " << joint_names_[i] <<
          "\ndir: "   << pin_dirs_[i] <<
          "\nstep: "  << pin_steps_[i]);
        break;

      default:
        ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
          "Joint mode not implemented");
        break;
    }
  }
}

}  // namespace kulbu_hardware
