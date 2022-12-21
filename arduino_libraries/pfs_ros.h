#include "pfs.h"

#undef ESP32
#include <ros.h>
#define ESP32

#include <pr2_fingertip_sensors/PR2FingertipSensor.h>

// ROS node handle.
ros::NodeHandle nh;
pr2_fingertip_sensors::PR2FingertipSensor pfs_msg;
#if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
  ros::Publisher pfs_pub("/pfs/from_uart", &pfs_msg);
#endif
#if (defined I2C_MASTER)
  ros::Publisher pfs_pub("/pfs/from_i2c", &pfs_msg);
#endif

void setup_nodehandle () {
  // Advertise ROS message
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(pfs_pub);
}

void set_pfs_fields(struct pfs_sensors* sensors,
                    pr2_fingertip_sensors::PR2FingertipSensor* pfs_msg) {
  // Proximity
  pfs_msg->proximity_length = NUM_SENSORS;
  pfs_msg->proximity = sensors->proximities;
  // Force
  pfs_msg->force_length = NUM_SENSORS;
  pfs_msg->force = sensors->forces;
  // IMU
  pfs_msg->imu.angular_velocity.x = sensors->gyro[0];
  pfs_msg->imu.angular_velocity.y = sensors->gyro[1];
  pfs_msg->imu.angular_velocity.z = sensors->gyro[2];
  pfs_msg->imu.linear_acceleration.x = sensors->acc[0];
  pfs_msg->imu.linear_acceleration.y = sensors->acc[1];
  pfs_msg->imu.linear_acceleration.z = sensors->acc[2];
  return;
}

void get_pfs_msg (pr2_fingertip_sensors::PR2FingertipSensor* pfs_msg, uint8_t pfs_address = 0x01) {
  // Get sensor data
  // If you use HARD/SOFTWARE Serial, stop them before publishing rostopic
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    begin_pfs();
  #endif
  struct pfs_sensors sensors;
  read_sensors(&sensors, pfs_address);
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    end_pfs_serial();
  #endif

  set_pfs_fields(&sensors, pfs_msg);
  pfs_msg->header.stamp = nh.now();
  char frame_id[15];
  sprintf(frame_id, "pfs_link_%d", pfs_address);
  pfs_msg->header.frame_id = frame_id;
}

void publish_pfs () {
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    // Make sure that other Serial is not used during rosserial communication
    get_pfs_msg(&pfs_msg);
    pfs_pub.publish(&pfs_msg);
  #endif
  #if (defined I2C_MASTER)
    for(int i=0; i<sizeof(pfs_addresses)/sizeof(uint8_t); i++) {
      get_pfs_msg(&pfs_msg, pfs_addresses[i]);
      pfs_pub.publish(&pfs_msg);
    }
  #endif
  nh.spinOnce();
}
