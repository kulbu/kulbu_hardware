#include <kulbu_hardware/kulbu_hardware_sysfs.h>

namespace kulbu_hardware {

KulbuHardwareSysfs::KulbuHardwareSysfs() {
}

  int KulbuHardwareSysfs::pwm_enable(unsigned int pwm, bool enable) {
    int fd;
    char buf[MAX_BUF];

    // First check current value, only update if changed.
    char curr[11];
    int status;
    snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/enable%d", pwm);
    fd = open(buf, sizeof(buf), O_RDONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
        "pwm/enable/read");
      return fd;
    }
    read(fd, curr, sizeof(curr));
    close(fd);

    // "On" always ends with an "n". Quick and dirty.
    status = (curr[strlen(curr)-2] == 'n');

    if (status != enable) {
      ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
         "Chan: " << pwm << " Enable: " << enable);

      snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/enable%d", pwm);
      fd = open(buf, O_WRONLY);
      if (fd < 0) {
        ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
          "pwm/enable/write");
        return fd;
      }

      if (enable)
        write(fd, "1", 2);
      else
        write(fd, "0", 2);

      close(fd);
    }
    return 0;
  }

  int KulbuHardwareSysfs::pwm_freq(unsigned int pwm, unsigned int freq) {
    int fd;
    char buf[MAX_BUF];

    // First check current value, only update if changed.
    char curr[11];
    int status;
    snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/freq%d", pwm);
    fd = open(buf, sizeof(buf), O_RDONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
        "pwm/freq/read");
      return fd;
    }
    read(fd, curr, sizeof(curr));
    close(fd);

    if (atoi(curr) != freq && freq >= 1) {
      ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
         "Chan: " << pwm << " Freq: " << freq);

      snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/freq%d", pwm);
      fd = open(buf, O_WRONLY);
      if (fd < 0) {
        ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
          "pwm/freq/write");
        return fd;
      }

      int len = snprintf(buf, sizeof(buf), "%d", freq);
      write(fd, buf, len);
      close(fd);
    }
    return 0;
  }

  int KulbuHardwareSysfs::pwm_duty(unsigned int pwm, unsigned int duty) {
    int fd;
    char buf[MAX_BUF];

    ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
       "Chan: " << pwm << " Duty: " << duty);

    snprintf(buf, sizeof(buf), SYSFS_PWM_DIR  "/duty%d", pwm);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
        "pwm/duty/write");
      return fd;
    }

    int len = snprintf(buf, sizeof(buf), "%d", duty);
    write(fd, buf, len);
    close(fd);

    return 0;
  }

  /*
  int gpio_own(char *file) {
    uid_t uid = getuid();
    uid_t gid = getgid();

    if (chown (file, uid, gid) != 0) {
      if (errno == ENOENT) {
        ROS_WARN_STREAM_NAMED("kulbu_hardware_interface",
          "gpio/own File does not exist");
      } else {
        ROS_FATAL_STREAM_NAMED("kulbu_hardware_interface",
          "gpio/own Unable to change ownership "
          << file << " Error: " << strerror (errno));
        exit (1) ;
      }
    }

  }

  int gpio_export(unsigned int gpio) {
    int fd;
    char buf[MAX_BUF];

    ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
       "Export: " << gpio);

    fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
        "gpio/export");
      return fd;
    }

    int len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
    gpio_own(buf);

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
    gpio_own(buf);

   return 0;
  }

  int gpio_dir(unsigned int gpio, bool output) {
    int fd;
    char buf[MAX_BUF];

    ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
       "Dir: " << gpio << " Output: " << output);

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
        "gpio/direction");
      return fd;
    }

    if (output)
      write(fd, "out", 2);
    else
      write(fd, "in", 2);

    close(fd);

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/direction", gpio);
    gpio_own(buf);

    return 0;
  }
  */

  int KulbuHardwareSysfs::gpio_set(unsigned int gpio, unsigned int value) {
    int fd;
    char buf[MAX_BUF];

    ROS_DEBUG_STREAM_NAMED("kulbu_hardware_interface",
       "GPIO: " << gpio << " Value: " << value);

    snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
    fd = open(buf, O_WRONLY);
    if (fd < 0) {
      ROS_ERROR_STREAM_NAMED("kulbu_hardware_interface",
        "gpio/set-value");
      return fd;
    }

    if (value)
      write(fd, "1", 2);
    else
      write(fd, "0", 2);

    close(fd);
    return 0;
  }
}  // namespace kulbu_hardware
