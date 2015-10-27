namespace kulbu_hardware {
  class KulbuHardwareMotor {
    public:
      /**
       * \brief Constructor
       */
      KulbuHardwareMotor();

      void init();
      void setJointVelocity(int index, unsigned int freq);
      void getJointVelocity(int index);

  }
}  // namespace kulbu_hardware
