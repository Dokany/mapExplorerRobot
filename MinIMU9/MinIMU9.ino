/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/
#define IMU_V5

// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
//int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>

#include <LSM6.h>
#include <LIS3MDL.h>

#define GRAVITY 9.81  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// gyro sense.: 125 deg/s
#define Gyro_Gain 0.004375 //Gyro gain dps/LSB

// acc sense.: 2g
#define Acc_Gain 0.000061 //Acc gain g/LSB


#define STATUS_LED 13

float g_OFFSET_x=0;
float g_OFFSET_y=0;
float g_OFFSET_z=0;
float a_OFFSET_x=0;
float a_OFFSET_y=0;
float a_OFFSET_z=0;
float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
long timer=0; 
long timer_old=timer;
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

unsigned int counter=0;
float G_Dt=0.02;  
const float samples=500.0;

LSM6 gyro_acc;

void setup()
{
  Serial.begin(115200);
  pinMode (STATUS_LED,OUTPUT);  // Status LED

  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);
  
  g_OFFSET_x=0;
  g_OFFSET_y=0;
  g_OFFSET_z=0;
  a_OFFSET_x=0;
  a_OFFSET_y=0;
  a_OFFSET_z=0;
   Accel_Init();
  Gyro_Init();
  
  for(int i=0;i<samples;i++)    // We take some readings...
    {
     // Gyro_Init();
      gyro_acc.readGyro();  
      g_OFFSET_x =  ToRad(gyro_acc.g.x*Gyro_Gain)+g_OFFSET_x;
      g_OFFSET_y =  ToRad(gyro_acc.g.y*Gyro_Gain)+g_OFFSET_y;
      g_OFFSET_z =  ToRad(gyro_acc.g.z*Gyro_Gain)+g_OFFSET_z;    
      
     //Accel_Init();      
      gyro_acc.readAcc();
      a_OFFSET_x =  (gyro_acc.a.x*GRAVITY*Acc_Gain)+a_OFFSET_x;
      a_OFFSET_y =  (gyro_acc.a.y*GRAVITY*Acc_Gain)+a_OFFSET_y;
      a_OFFSET_z =  (gyro_acc.a.z*GRAVITY*Acc_Gain)+a_OFFSET_z;
   
      delay(20);
    }

  g_OFFSET_x=g_OFFSET_x/samples;
  g_OFFSET_y=g_OFFSET_y/samples;
  g_OFFSET_z=g_OFFSET_z/samples;
  a_OFFSET_x=a_OFFSET_x/samples;
  a_OFFSET_y=a_OFFSET_y/samples;
  a_OFFSET_z=GRAVITY+(a_OFFSET_z/samples);
  
  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  timer=millis();
  delay(20);
  counter=0;
}

void loop() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
    {
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
      if (G_Dt > 0.2)
        G_Dt = 0; // ignore integration times over 200 ms
    }
    else
      G_Dt = 0;

    //Gyro_Init();

    gyro_acc.readGyro();  
    gyro_x =  ToRad(gyro_acc.g.x*Gyro_Gain)-g_OFFSET_x;
    gyro_y =  ToRad(gyro_acc.g.y*Gyro_Gain)-g_OFFSET_y;
    gyro_z =  ToRad(gyro_acc.g.z*Gyro_Gain)-g_OFFSET_z;
    Serial.print("! GYRO x=");
    Serial.print(gyro_x);
    Serial.print(" y=");
    Serial.print(gyro_y);
    Serial.print(" z=");
    Serial.print(gyro_z);

    //Accel_Init();
    
    gyro_acc.readAcc();
    accel_x =  (gyro_acc.a.x*GRAVITY*Acc_Gain)-a_OFFSET_x;
    accel_y =  (gyro_acc.a.y*GRAVITY*Acc_Gain)-a_OFFSET_y;
    accel_z =  (gyro_acc.a.z*GRAVITY*Acc_Gain)-a_OFFSET_z;
    Serial.print("! ACCEL x=");
    Serial.print(accel_x);
    Serial.print(" y=");
    Serial.print(accel_y);
    Serial.print(" z=");
    Serial.println(accel_z);
    
   // Serial.println(gyro_acc.a.x);
   // Serial.println(gyro_acc.a.y);
   // Serial.println(gyro_acc.a.z);
    delay(100);
 }

}



