#include <kulbu_hardware/kulbu_hardware_sysfs.h>

namespace kulbu_hardware {
  class KulbuHardwareRobot {
    public:
      /**
       * \brief Constructor
       */
      KulbuHardwareRobot();

      std::vector<int>    pin_dirs;
      std::vector<int>    pin_steps;

      void initJointVelocity(int index);
      void setJointVelocity(int index, unsigned int freq);
      unsigned int getJointVelocity(int index);

  }
}  // namespace kulbu_hardware
