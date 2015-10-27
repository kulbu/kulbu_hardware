#include <kulbu_hardware/kulbu_hardware_robot.h>

namespace kulbu_hardware {

  KulbuHardwareRobot::KulbuHardwareRobot() {
    ROS_INFO_NAMED("kulbu_hardware_robot",
      "Loaded kulbu_hardware_robot.");
  }

  void KulbuHardwareRobot::initJointVelocity(int index) {
    // Export GPIO pin for output.
    // FIXME: Permission denied? Manual export `gpio export 88 out`
    // gpio_export(pin_dirs[i]);
    // gpio_dir(pin_dirs[i], 1);

    // Start with PWM turned off and 50/50 duty cycle.
    KulbuHardwareSysfs::pwm_enable(pin_steps[index], 0);
    KulbuHardwareSysfs::pwm_duty(pin_steps[index], 512);

    // Set direction pins to forward by default.
    KulbuHardwareSysfs::gpio_set(pin_dirs[index], 0);
  };

  void KulbuHardwareRobot::setJointVelocity(int index, unsigned int freq) {
    int dir = 0;

    // Reverse flips direction.
    if (freq < 0) {
      freq = abs(freq);
      dir = 1;
    } else {
      dir = 0;
    }

    // Disable PWM when stopped.
    if (freq < 1) {
      freq = 0;
      KulbuHardwareSysfs::pwm_enable(pin_steps[index], 0);
    } else {
      KulbuHardwareSysfs::pwm_enable(pin_steps[index], 1);
    }

    // Set direction pin.
    KulbuHardwareSysfs::gpio_set(pin_dirs[index], dir);

    // Set PWM frequency.
    KulbuHardwareSysfs::pwm_freq(pin_steps[index], freq);
  };

  unsigned int KulbuHardwareRobot::getJointVelocity(int index) {

  };
}  // namespace kulbu_hardware
