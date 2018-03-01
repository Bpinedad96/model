
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>

char *joints[] = {"base_link_to_base_pitch", "base_pitch_to_base_heading", "base_heading_to_chasis", "chasis_to_right", "chasis_to_left", "chasis_to_front_right", "chasis_to_front_left" };
float positions[7];
float pastPositions[7];

ros::NodeHandle nh;
sensor_msgs::JointState  link_state;
ros::Publisher joint_states("joint_states", &link_state);

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);

/* Update this with the correct SLP for accurate altitude measurements */

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    //Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    //while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    //Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    //while (1);
  }
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void setup()
{
  nh.initNode();
  nh.advertise(joint_states);
  link_state.name_length = 7;
  link_state.position_length = 7;

  /* Initialise the sensors */
  initSensors();

  /*for(i=0;i<4;i++){
    lis[i] = Adafruit_LIS3DH(LIS3DH_CS[i], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
    if (! lis[i].begin(0x18)) {   
      while (1);
    } 
    lis[i].setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  }*/
}

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/
void loop()
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  float radsPerChange=0.0523;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {    
      //if ( ( ( (-orientation.roll)/50)/radsPerChange)%3)
      positions[2]=(-orientation.roll)/50;//50 y 70  
      positions[1]=orientation.pitch/50;  
      
      //positions[0] = map((orientation.roll), 0, 360, -90, 90)/100;
      //positions[1] = map((orientation.pitch+3.2)*10, 0, 1023, -350, 350) / 100;
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {    
     positions[0]=-orientation.heading/50;  
     //positions[2] = map(orientation.heading*1, 0, 1023, -350, 350) / 100;
  }

  link_state.header.stamp = nh.now();
  link_state.name = joints;
  link_state.position = positions;

  joint_states.publish(&link_state);
  
  nh.spinOnce();
  //delay(50);
}


