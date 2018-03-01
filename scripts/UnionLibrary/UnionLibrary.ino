#include "src/Imu/Imu.h"
#include "ros.h"
#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>

//Joint information
char *joints[] = {"base_link_to_base_pitch", "base_pitch_to_base_roll", "base_roll_to_chasis", "chasis_to_right", "chasis_to_left", "chasis_to_right2", "chasis_to_left2" };
float positions[7];

//ROS nodes
ros::NodeHandle nh;
sensor_msgs::JointState  link_state;
ros::Publisher joint_states("joint_states", &link_state);

Imu imu(0);

/* Update this with the correct SLP for accurate altitude measurements */

/**************************************************************************/

void setup()
{
  nh.initNode();
  nh.advertise(joint_states);
  link_state.name_length = 7;
  link_state.position_length = 7;

  /* Initialise the sensors */
  imu.initSensors();

}

void loop()
{

    imu.updateData();

    positions[2]=imu.getRoll();
    positions[1]=imu.getPitch();
    positions[0]=imu.getYaw();  
    positions[3]=imu.getWheel1();
    positions[4]=imu.getWheel2();
    positions[5]=imu.getWheel3();
    positions[6]=imu.getWheel4();

  link_state.header.stamp = nh.now();
  link_state.name = joints;
  link_state.position = positions;

  joint_states.publish(&link_state);
  
  nh.spinOnce();
  delay(50);
}


