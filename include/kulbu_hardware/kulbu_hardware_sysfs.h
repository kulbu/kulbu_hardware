#ifndef KULBU_ROS_CONTROL__KULBU_HARDWARE_SYSFS_H
#define KULBU_ROS_CONTROL__KULBU_HARDWARE_SYSFS_H

//#include <ros/ros.h>
#include <fcntl.h>

#define SYSFS_GPIO_DIR  "/sys/class/gpio"
#define SYSFS_PWM_DIR   "/sys/devices/platform/pwm-ctrl"
#define MAX_BUF 256

namespace kulbu_hardware {

  class KulbuHardwareSysfs {
    public:
      /**
       * \brief Constructor
       */
      KulbuHardwareSysfs();

      int pwm_enable(unsigned int pwm, bool enable);
      int pwm_freq(unsigned int pwm, unsigned int freq);
      int pwm_duty(unsigned int pwm, unsigned int duty);
      int gpio_set(unsigned int gpio, unsigned int value);
  }
}  // namespace kulbu_hardware

#endif  // KULBU_ROS_CONTROL__KULBU_HARDWARE_SYSFS_H
