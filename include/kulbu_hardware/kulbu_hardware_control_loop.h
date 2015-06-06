/**
 * Based on: https://github.com/davetcoleman/ros_control_boilerplate
 */

#include <time.h>
#include <kulbu_hardware/kulbu_hardware_interface.h>

namespace kulbu_hardware {

// Used to convert seconds elapsed to nanoseconds
static const double BILLION = 1000000000.0;

/**
 * \brief The control loop - repeatidly calls read() and write() to the hardware interface at a specified frequency
 *        We use MONOTONIC time to ensure robustness in the event of system time updates/change.
 *        See http://stackoverflow.com/questions/3523442/difference-between-clock-realtime-and-clock-monotonic
 */
class KulbuHardwareControlLoop {
 public:
  /**
   * \brief Constructor
   * \param NodeHandle
   * \param hardware_interface - the robot-specific hardware interface to be use with your robot
   */
  KulbuHardwareControlLoop(ros::NodeHandle& nh,
    boost::shared_ptr<kulbu_hardware::KulbuHardwareInterface> hardware_interface);

  /** \brief Timer event
   *         Note: we do not use the TimerEvent time difference because it does NOT guarantee that the time source is
   *         strictly linearly increasing
   */
  void update(const ros::TimerEvent& e);

 protected:
  // Startup and shutdown of the internal node inside a roscpp program
  ros::NodeHandle nh_;

  // Timing
  ros::Timer      non_realtime_loop_;
  ros::Duration   elapsed_time_;
  double          loop_hz_;
  struct timespec last_time_;
  struct timespec current_time_;

  /** \brief ROS Controller Manager and Runner
   *
   * This class advertises a ROS interface for loading, unloading, starting, and
   * stopping ros_control-based controllers. It also serializes execution of all
   * running controllers in \ref update.
   */
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  /** \brief Abstract Hardware Interface for your robot */
  boost::shared_ptr<kulbu_hardware::KulbuHardwareInterface> hardware_interface_;
};  // end class

}  // namespace kulbu_hardware
