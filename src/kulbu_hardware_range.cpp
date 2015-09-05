#include <ros/ros.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

int i2c_bus;
int i2c_slave_address;
std::vector<int> i2c_registers;

int main(int argc, char** argv) {
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
  // TODO: configurable rate.
  ros::Rate r(20.0);
  while (n.ok()) {
    // Open the i2c bus.
    int fh;
    char filename[20];
    snprintf(filename, 19, "/dev/i2c-%d", i2c_bus);
    fh = open(filename, O_RDWR);
    if (fh < 0) {
      ROS_ERROR_STREAM("kulbu_hardware_range: Failed opening i2c_bus=" << filename);
      exit(1);
    }

    // Lookup device by `i2c_slave_address`.
    if (ioctl(fh, I2C_SLAVE, i2c_slave_address) < 0) {
      ROS_ERROR_STREAM("kulbu_hardware_range: Failed opening i2c_slave_address=" << i2c_slave_address);
      exit(1);
    }

    for(unsigned i=0; i < i2c_registers.size(); i++) {
      // Lookup result in `i2c_registers[i]`.
      int32_t res;
      char buf[10];

      res = i2c_smbus_read_word_data(fh, i2c_registers[i]);
      if (res < 0) {
        ROS_ERROR_STREAM("kulbu_hardware_range: Failed i2c transaction i2c_register=" << i2c_registers[i]);
        // continue
      } else {
        ROS_DEBUG_STREAM("kulbu_hardware_range: reg=" << i2c_registers[i] << " res=" << res);
      }
    }

    //ros::spin();
    ros::spinOnce();
    r.sleep();
  }
}
