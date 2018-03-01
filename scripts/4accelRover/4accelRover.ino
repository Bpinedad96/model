// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11

// Used for hardware & software SPI
const int LIS3DH_CS[4] = {7,8,9,10};
int i = 0;
Adafruit_LIS3DH lis[4];

// software SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS[0], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// hardware SPI
//Adafruit_LIS3DH lis = Adafruit_LIS3DH(LIS3DH_CS);
// I2C
//Adafruit_LIS3DH lis = Adafruit_LIS3DH();

///#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   //#define Serial SerialUSB
//#endif

void setup(void) {
  for(i=0;i<4;i++){
lis[i] = Adafruit_LIS3DH(LIS3DH_CS[i], LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);
//#ifndef ESP8266
  //while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
//#endif
//Serial.begin(9600);
  //Serial.print("LIS3DH ");Serial.print(i+1);Serial.print(" test!\n");
  if (! lis[i].begin(0x18)) {   // change this to 0x19 for alternative i2c address
    //Serial.print("Couldnt start sensor#");Serial.print(i+1);Serial.println("\n");
    while (1);
  }
  //Serial.print("LIS3DH ");Serial.print(i+1);Serial.print(" found!\n");
  
  lis[i].setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!
  
  //Serial.print("Range S");Serial.print(i+1);Serial.print(" ");Serial.print(2 << lis[i].getRange());  
  //Serial.print("G\n");
  }

}

void loop() {
for(i=0;i<4;i++){
  lis[i].read();      // get X Y and Z data at once
  // Then print out the raw data
  //Serial.print("X");Serial.print(i+1);Serial.print(" "); Serial.print(lis[i].x); 
  Serial.print("Y");Serial.print(i+1);Serial.print(" "); Serial.print(lis[i].y); 
  //Serial.print("\tZ");Serial.print(i+1);Serial.print(" "); Serial.print(lis[i].z); 

  //Y4 y Y3 por su pedo
  /* Or....get a new sensor event, normalized */ 
  sensors_event_t event; 
  lis[i].getEvent(&event);
  
  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("\tX: "); Serial.print(event.acceleration.x);
  //Serial.print("\tY: "); Serial.print(i+1);Serial.print(" "); Serial.print(event.acceleration.y); 
  //Serial.print("\tZ: "); Serial.print(i+1);Serial.print(" ");Serial.print(event.acceleration.z); 
  //Serial.println(" m/s^2 ");

  Serial.println();
 
  delay(100); 
  }
}
