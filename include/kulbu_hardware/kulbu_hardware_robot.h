namespace kulbu_hardware {
  class KulbuHardwareRobot {
    public:
      /**
       * \brief Constructor
       */
      KulbuHardwareRobot();

      void init();
      void setJointVelocity(int index, unsigned int freq);
      void getJointVelocity(int index);

  }
}  // namespace kulbu_hardware
