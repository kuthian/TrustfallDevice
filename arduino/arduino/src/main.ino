#include <MPU9250.h>
#include <quaternionFilters.h>
#include "MPU9250.h"            // Sparkfun Library
#include <SoftwareSerial.h>     // Bluetooth Serial Connection
#include<AltSoftSerial.h>

MPU9250 myIMU;                  // Define the I2C address for the MPU

float run_vx = 0;
float run_vy = 0;
float run_vz = 0;
float run_vxy = 0;
float run_vxz = 0;
float run_vyz = 0;
float run_vxyz = 0;

float dx = 0; 
float dy = 0;
float dz = 0;
float dxy = 0;
float dxz = 0;
float dyz = 0;
float dxyz = 0;

float vx;
float vy;
float vz;
float vxy;
float vxz;
float vyz;
float vxyz;

int i = 1;
int N = 100;
int low_batt_flag = 0;
int event_trigger_flag = 0;

int LedPin = 13; // Pin 13 is the arduino pro mini LED pin

unsigned long int now = 0; // Set up variable for the timer
unsigned long int last_fall_time = 0; // var that holds time of last fall
unsigned long int last_batt_check_time = 0; // var that holds time of last fall

float threshold = 3.8; // threshold in G 
float threshold2 = 5.5; // threshold in G

AltSoftSerial BT;

void setup()
{
  int IntPin = 2; //These can be changed, 2 and 3 are the Arduinos ext int pins
  int SoftSerialPin = 9; 
  
  pinMode(A0, INPUT);
  pinMode(IntPin, INPUT);           // Set up the interrupt pin
  pinMode(LedPin, OUTPUT);          // LED pin
  pinMode(SoftSerialPin, OUTPUT);   // Needed for SoftSerial not sure why


  BT.begin(9600);               // set up bluetooth connection, start advertising
  Serial.begin(38400);          // set up Serial connection
  Wire.begin();                 // init I2C connection
  
  digitalWrite(IntPin, LOW);    // on digital write this pin is held low preventing interupt 

  // This is checking the MPU addess default is 71
  if (myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250) == 0x71)
  {
    uint8_t SmpRtDiv = 0x09;        // This is the number the fundamental freq of MPU sampler is divided by +1 the default fund freq is 1kHz IE for a 100Hz refresh SmpRtDiv = 9
    Serial.println("MPU9250 is online...");
    
    myIMU.MPU9250SelfTest(myIMU.SelfTest); //Check Biases
    
    //myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias); //Load Biases into registers

    myIMU.initMPU9250();
    myIMU.writeByte(MPU9250_ADDRESS, SMPLRT_DIV, SmpRtDiv); // Sets the MPU sample rate to 100Hz
        
    Serial.println("MPU9250 initialized...");
    
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250), HEX);                          // Print address of MPU
    while(1) ;                                       // Loop forever if communication doesn't happen
  }
}

void loop()
{       
  // This loops until the the MPU Int data register goes high, This will go high only when every data register contains new data 
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
      //Reading from MPU9250
      now = millis(); 
      // records how long the MPU has been running in milliseconds
      myIMU.readAccelData(myIMU.accelCount);            // Read the x/y/z adc values
      myIMU.getAres();                                  // Sets the DAC resolution
  
      myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // records MPU x axis  
      myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // records MPU y axis
      myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // records MPU z axis
  
      //These are single datapoints for computing moving average
      vx = magfunction1(myIMU.ax);
      vy = magfunction1(myIMU.ay);
      vz = magfunction1(myIMU.az);
      vxy = magfunction2(myIMU.ax,myIMU.ay);
      vxz = magfunction2(myIMU.ax,myIMU.az);
      vyz = magfunction2(myIMU.ay,myIMU.az);
      vxyz = magfunction3(myIMU.ax,myIMU.ay,myIMU.az);

      if(i>100)
      {
        //first time over 100 samples are reached, obtain baseline average
        if(i == 101)
        {
          run_vx = run_vx/100;
          run_vy = run_vy/100;
          run_vz = run_vz/100;
          run_vxy = run_vxy/100;
          run_vxz = run_vxz/100;
          run_vyz = run_vxz/100;
          run_vxyz = run_vxyz/100;

          i++;
        }
        
        run_vx = (run_vx) + (vx/N) - (run_vx/N);
        run_vy = run_vy + vy/N - run_vy/N;
        run_vz = run_vz + vz/N - run_vz/N;
        run_vxy = run_vxy + vxy/N - run_vxy/N;
        run_vxz = run_vxz + vxz/N - run_vxz/N;
        run_vyz = run_vyz + vyz/N - run_vyz/N;
        run_vxyz = run_vxyz + vxyz/N - run_vxyz/N;
  
        dx = delta(vx, run_vx); 
        dy = delta(vy, run_vy);
        dz = delta(vz, run_vz);
        dxy = delta(vxy, run_vxy);
        dxz = delta(vxz, run_vxz);
        dyz = delta(vyz, run_vyz);
        dxyz = delta(vxyz, run_vxyz);


        if((low_batt_flag = 0) && ((now - last_batt_check_time) >= 10000))
        {
          int batt = analogRead(A0);
          float voltage = batt * (4.2/1023);
          if(voltage < 3.6)
          {
            low_batt_flag = 1;
            BT.print("BATTERY LOW");
          }
          last_batt_check_time = millis(); 
        }

        if ((now - last_fall_time) >= 10000)
        {
          if ((abs(dx) > threshold) || (abs(dy) > threshold) || (abs(dz) > threshold) || (abs(dxy) > threshold) || (abs(dxz) > threshold) || (abs(dyz) > threshold) || (abs(dxyz) > threshold2))
          {
            
            event_trigger_flag = 1;
            digitalWrite(LedPin, HIGH); 
            BT.print("EventTrigger");
            last_fall_time = millis(); 
          }
          else if (event_trigger_flag == 1)
          {
            event_trigger_flag = 0;
            digitalWrite(LedPin, 0); 
          }
          else
          {
            event_trigger_flag = 0;
          }
        }
              //myIMU.updateTime();      MIGHT NOT BE NEEDED LOL
      }
      else
      {
        //first 100 samples will not have a moving average
        run_vx = run_vx + vx;
        run_vy = run_vy + vy;
        run_vz = run_vz + vz;
        run_vxy = run_vxy + vxy;
        run_vxz = run_vxz + vxz;
        run_vyz = run_vyz + vyz;
        run_vxyz = run_vxyz + vxyz;

        i++;
      }  
  }
}

float magfunction1(float one){
  float mags = sqrt(sq(one));
  return mags;
}

float magfunction2(float one, float two){
  float mags = sqrt(sq(one)+sq(two));
  return mags;
}

float magfunction3(float one, float two, float three){
  float mags = sqrt(sq(one)+sq(two)+sq(three));
  return mags;
}

//Calculates delta value
float delta(float vx, float run_vx)
{
  float delta = vx-run_vx;
  return delta;
}

