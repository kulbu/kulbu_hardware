namespace kulbu_hardware {
  class KulbuHardwareMotor {
    public:
      /**
       * \brief Constructor
       */
      KulbuHardwareMotor();

      void init();
      void move(int dir, int freq);

  }
}  // namespace kulbu_hardware
