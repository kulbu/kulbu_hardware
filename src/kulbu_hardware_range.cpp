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
  pn.param("i2c_slave_address", i2c_slave_address, 0);
  pn.getParam("i2c_registers", i2c_registers); // FIXME: default?

  ROS_INFO_STREAM("kulbu_hardware_range: Parameters i2c_bus=" << i2c_bus << " i2c_slave_address=" << i2c_slave_address << " i2c_registers=" << i2c_registers.size());

  if (!(i2c_slave_address > 3)) {
    ROS_ERROR_STREAM("kulbu_hardware_range: No address specified.");
    exit(1);
  }

  if (i2c_registers.size() == 0) {
    ROS_ERROR_STREAM("kulbu_hardware_range: No registers specified.");
    exit(1);
  }

  // Open the i2c bus.
  int fh;
  char filename[20];
  snprintf(filename, 19, "/dev/i2c-%d", i2c_bus);
  fh = open(filename, O_RDWR);
  if (fh < 0) {
    ROS_ERROR_STREAM("kulbu_hardware_range: Failed opening i2c_bus=" << filename);
    exit(1);
  }
  //ROS_INFO_STREAM("kulbu_hardware_range: Bus opened i2c_bus=" << filename);

  // Lookup device by `i2c_slave_address`.
  if (ioctl(fh, I2C_SLAVE, i2c_slave_address) < 0) {
    ROS_ERROR_STREAM("kulbu_hardware_range: Failed opening i2c_slave_address=" << i2c_slave_address);
    exit(1);
  }

  // Main loop
  //ros::spin();
  // TODO: configurable rate.
  ros::Rate r(20.0);
  while (n.ok()) {
      /*
      int res;
      res = i2c_smbus_read_word_data(fh, 0x00);
      if (res < 0) {
        ROS_ERROR_STREAM("kulbu_hardware_range: Failed i2c transaction");
        // continue
      } else {
        ROS_INFO_STREAM("kulbu_hardware_range: res=" << res);
      }
      */

      /*
      int res;
      res = i2c_smbus_read_byte_data(fh, 0x00);
      if (res < 0) {
        ROS_ERROR_STREAM("kulbu_hardware_range: Failed i2c transaction");
        // continue
      } else {
        ROS_INFO_STREAM("kulbu_hardware_range: res=" << res);
      }
      */

      /*
      int res;
      unsigned char cblock[288];
      res = i2c_smbus_read_block_data(fh, 0x00, cblock);
      if (res < 0) {
        ROS_ERROR_STREAM("kulbu_hardware_range: Failed i2c transaction");
        // continue
      } else {
        ROS_INFO_STREAM("kulbu_hardware_range: a res=" << res);
        int xi;
        for (xi = 0; xi < res+1; xi++) {
          ROS_INFO_STREAM("kulbu_hardware_range: idx=" << xi << " blk=" << cblock[xi]);
        }
      }
      */

    for(unsigned i=0; i < i2c_registers.size(); i++) {
      //ROS_INFO_STREAM("kulbu_hardware_range: reg=" << i2c_registers[i]);
      // Lookup result in `i2c_registers[i]`.
      int res;
      res = i2c_smbus_read_word_data(fh, i2c_registers[i]);
      if (res < 0) {
        ROS_ERROR_STREAM("kulbu_hardware_range: Failed i2c transaction i2c_register=" << i2c_registers[i]);
        // continue
      } else {
        ROS_INFO_STREAM("kulbu_hardware_range: reg=" << i2c_registers[i] << " res=" << res);
      }
    }

    //ros::spin();
    ros::spinOnce();
    r.sleep();
  }

  // TODO: Cleanup and close file.
}
