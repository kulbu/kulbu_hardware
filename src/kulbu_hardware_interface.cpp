/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#include <kulbu_hardware/kulbu_hardware_interface.h>
#include <fcntl.h>
// #include <wiringPi.h>
// #include <softTone.h>

#define SYSFS_GPIO_DIR  "/sys/class/gpio"
#define SYSFS_PWM_DIR   "/sys/devices/platform/pwm-ctrl"
#define MAX_BUF 256

// Temp place-holder for some sysfs code.
int pwm_status(unsigned int pwm, bool enable) {
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/enable%d", pwm);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    perror("pwm/enable");
    return fd;
  }

  if (enable)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);

  close(fd);
  return 0;
}

int pwm_freq(unsigned int pwm, unsigned int freq) {
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/freq%d", pwm);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    perror("pwm/freq");
    return fd;
  }

  len = snprintf(buf, sizeof(buf), "%d", freq);
  write(fd, buf, len);
  close(fd);

  return 0;
}

int gpio_set(unsigned int pin, unsigned int value) {
  int fd;
  char buf[MAX_BUF];

  snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);

  fd = open(buf, O_WRONLY);
  if (fd < 0) {
    perror("gpio/set-value");
    return fd;
  }

  if (value)
    write(fd, "1", 2);
  else
    write(fd, "0", 2);

  close(fd);
  return 0;
}

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

  nh_.getParam("hardware_interface/dirs", pin_dirs_);
  if (pin_dirs_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
      "No GPIO direction pins found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  nh_.getParam("hardware_interface/steps", pin_steps_);
  if (pin_steps_.size() == 0) {
    ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
      "No GPIO step pins found on parameter server."
        << " Namespace: " << nh_.getNamespace());
    exit(-1);
  }

  // Resize vectors
  joint_position_.resize(num_joints_);
  joint_velocity_.resize(num_joints_);
  joint_effort_.resize(num_joints_);
  joint_position_command_.resize(num_joints_);
  joint_velocity_command_.resize(num_joints_);
  joint_effort_command_.resize(num_joints_);

  // Initialise GPIO
  // wiringPiSetupSys();
  // FIXME: Ensure duty cycle is correct.
  // `512 > /sys/devices/platform/pwm-ctrl/duty0`

  // Initialize controller
  for (std::size_t i = 0; i < num_joints_; ++i) {
    ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
      "Loading joint name: " << joint_names_[i]);

    // Set direction pins to forward by default.
    // pinMode(pin_dirs_[i], OUTPUT);
    // digitalWrite(pin_dirs_[i], LOW);

    // Create PWM tone generator for joint
    // pinMode(pin_steps_[i], SOFT_TONE_OUTPUT);
    // softToneCreate(pin_steps_[i]);

    // Start with PWM turned off.
    pwm_status(pin_steps_[i], 0);
    // Start going forward.
    gpio_set(pin_dirs_[i], 0);

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
  int freq = 0;
  int dir = 0;

  // Send commands in different modes
  for (std::size_t i = 0; i < num_joints_; ++i) {
    // Skip joints with no pins defined
    if (pin_dirs_[i] == -1 || pin_steps_[i] == -1) continue;

    switch (joint_mode_) {
      case 1:  // hardware_interface::MODE_VELOCITY:
        // Calc PWM frequency.
        freq = joint_velocity_command_[i] * steps_per_m_;

        // Reverse flips direction.
        if (freq < 0) {
          freq = abs(freq);
          dir = 1;
        } else {
          dir = 0;
        }

        // Disable PWM when stopped.
        if (freq < 1) {
          pwm_status(pin_steps_[i], 0);
        // FIXME: Only if previously deactivated.
        } else {
          pwm_status(pin_steps_[i], 1);
        }

        // Set direction pin
        // digitalWrite(pin_dirs_[i], dir);
        gpio_set(pin_dirs_[i], dir);

        // Set PWM frequency
        // softToneWrite(pin_steps_[i], freq);
        pwm_freq(pin_steps_[i], freq);

        ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
          "\ni: "     << i <<
          "\ncmd: "   << joint_velocity_command_[i] <<
          "\ndist: "  << steps_per_m_ <<
          "\nfreq: "  << freq <<
          "\ndir: "   << dir <<
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
