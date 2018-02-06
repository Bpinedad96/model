//#include <i2c_t3.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

//Accelerometers
#include <SPI.h>
#include <Adafruit_LIS3DH.h>

// Used for software SPI
#define LIS3DH_CLK 32
#define LIS3DH_MISO 1
#define LIS3DH_MOSI 0

#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>

//Joint information
char *joints[] = {"base_link_to_base_pitch", "base_pitch_to_base_roll", "base_roll_to_chasis", "chasis_to_right", "chasis_to_left", "chasis_to_right2", "chasis_to_left2" };
float positions[7];
float pastPositionsx[10];
float pastPositionsy[10];
float pastPositionsz[10];

// Used for hardware & software SPI
const int LIS3DH_CS[4] = {23, 22, 21, 20};
int i = 0;
int j = 0;
Adafruit_LIS3DH lis[4];

//ROS nodes
ros::NodeHandle nh;
sensor_msgs::JointState  link_state;
ros::Publisher joint_states("joint_states", &link_state);

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302); 

/* Update this with the correct SLP for accurate altitude measurements */

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

  for(i=0;i<4;i++){
    lis[i] = Adafruit_LIS3DH(LIS3DH_CS[i], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
    if (! lis[i].begin(0x18)) {   
      while (1);
    } 
    lis[i].setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  }
}

float promPast (float pastPositions[]) {
  float prom;
  for(i=0;i<10;i++){
    prom=pastPositions[i];
  }
  return prom/10;
  
}

void setpast (float pastPositions[]) {
  for(i=0;i<10;i++){
    pastPositions[i]=0;
  }
}

/**************************************************************************/
void setup()
{
//Wire.begin(38, 37); // similar to above, but using SCL pin 16 and SDA pin 18  i2c_t3.setSDA(37);
  //Wire.begin(I2C_MASTER, 0x00, 38, 37);

  nh.initNode();
  nh.advertise(joint_states);
  link_state.name_length = 7;
  link_state.position_length = 7;

  /* Initialise the sensors */
  initSensors();
  
  //setpast(pastPositionsx);
  //setpast(pastPositionsy);
  //setpast(pastPositionsz);
}

void loop()
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_vec_t   orientation;

  float radsPerChange=0.0523;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {    
      if ( abs( ((-orientation.roll)/50)-positions[2])>radsPerChange*2) { //and abs( ((-orientation.roll)/50)-positions[2])<radsPerChange*4) {
      //if ( abs( ((-orientation.roll)/50)-promPast(pastPositionsx))>radsPerChange*2 and abs( ((-orientation.roll)/50)-promPast(pastPositionsx))<radsPerChange*3)   {
                //pastPositionsx[j]=(-orientation.roll)/50;
                positions[2]=(-orientation.roll)/50;
      }
      
    if ( abs( ((-orientation.pitch)/50)-positions[1])>radsPerChange*2) { //and abs( ((-orientation.pitch)/50)-positions[1])<radsPerChange*4) {
   // if ( abs( ((-orientation.pitch)/50)-promPast(pastPositionsy))>radsPerChange*2 and abs( ((-orientation.pitch)/50)-promPast(pastPositionsy))<radsPerChange*3)   {
                //pastPositionsy[j]=(-orientation.pitch)/50;
        positions[1]=orientation.pitch/50;  
   }
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {    
     if ( abs( ((-orientation.heading)/50)-positions[0])>radsPerChange*2){ //and abs( ((-orientation.heading)/50)-positions[0])<radsPerChange*4)
      //pastPositionsz[j]=(-orientation.heading)/50;
        positions[0]=-orientation.heading/50;  
     }
  }

  for(i=0;i<4;i++){
    lis[i].read();      // get X Y and Z data at once
    if ( abs( ((lis[i].y/5095.54)-positions[i+3]))>radsPerChange*2) //and abs( ((lis[i].y/5095.54)-positions[i+3]))<radsPerChange*4 )
      positions[i+3]=(lis[i].y/5095.54); 
    //Y4 y Y3 por su pedo
    sensors_event_t event; 
    lis[i].getEvent(&event);
  }

  /*if (j==9)
    j=0;
  else
    j++;*/

  link_state.header.stamp = nh.now();
  link_state.name = joints;
  link_state.position = positions;

  joint_states.publish(&link_state);
  
  nh.spinOnce();
}


