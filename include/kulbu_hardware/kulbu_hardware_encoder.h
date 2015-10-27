namespace kulbu_hardware {
  class KulbuHardwareEncoder {
    public:
      /**
       * \brief Constructor
       */
      KulbuHardwareEncoder();

      void init();
      void getJointVelocity(int index);
  }
}  // namespace kulbu_hardware
