#include <ros/ros.h>
#include <linux/i2c-dev.h>

int i2c_bus;
int i2c_slave_address;
std::vector<int> i2c_registers;

int main( int argc, char** argv) {
  // Ros pub/sub.
  if (!ros::isInitialized()) {
    ros::init(argc, argv, "kulbu_hardware_range");
  }

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pn.param("i2c_bus", i2c_bus, 1);
  pn.param("i2c_slave_address", i2c_slave_address, 21); // 0x15
  pn.param("i2c_registers", i2c_registers); // FIXME: default?

  ROS_INFO_STREAM("kulbu_hardware_range: Parameters i2c_bus=" << i2c_bus << " i2c_slave_address=" << i2c_slave_address);

  // Main loop
  //ros::spin();
  ros::Rate r(20.0);
  while (n.ok()) {
    //ROS_DEBUG_STREAM("kulbu_hardware_range: ");

    // TODO: Open the `i2c_slave_address`.
    for(unsigned i=0; i < i2c_registers.size(); i++) {
      // TODO: Lookup result in `i2c_registers[i]`.
      //ROS_DEBUG_STREAM("kulbu_hardware_range: reg=" << i << " res=" << res);
    }

    //ros::spin();
    ros::spinOnce();
    r.sleep();
  }
}
