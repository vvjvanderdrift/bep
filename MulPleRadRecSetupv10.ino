// Receiver

//Include necessary libraries
#include <SPI.h>  
#include "RF24.h" 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//Define radio connecting pin and address for transceivers
RF24 myRadio (9, 8); 
byte addrs[][6] = {"T"}; 

// Define necessary constants and parameters for calculations and timing.
unsigned long currentMillis;
unsigned long previousMillis = 0;
float freq = 2;
float interval = 1000/freq;
float radius = 0.35;

// Define a package that contains 20 bytes of calibration data of the IMU's as well as the angular velocity & acceleration (u6 and u6_d). This package will be received over the radio modules.
struct package0{                
  uint32_t measurement = 0; // 4 bytes
  uint8_t system1 = 0; // 1 byte
  uint8_t gyros1 = 0; // 1 byte
  uint8_t acc1 = 0; // 1 byte
  uint8_t mg1 = 0; //1 byte 
  uint8_t system2 = 0; // 1 byte
  uint8_t gyros2 = 0; // 1 byte
  uint8_t acc2 = 0; // 1 byte
  uint8_t mg2 = 0; // 1 byte
  float u6_radio = 0; // 4 bytes
  float u6_d_radio = 0; // 4 bytes
};
typedef struct package0 Package0;
Package0 cali;

//Define a function that reads out the radiomodule and prints the necessary data to the serial monitor.
void receiveAndPrint() {
  if (myRadio.available()) {
    while (myRadio.available()) {        // Pick up transmission from transmitter.
      myRadio.read(&cali, sizeof(cali)); // Get the values of package cali from radio buffer.
    }
    
	//Print the calibration data and speed & acceleration for monitoring system
  	Serial.print(cali.measurement);Serial.println(" : Measurement"); //Measurement number IMU Q
	Serial.print(cali.system1);Serial.println(" : System calibration IMU Q (front)"); //System calibration IMU Q
  	Serial.print(cali.gyros1);Serial.println(" : Gyroscope calibration IMU Q (front)"); //Gyroscope calibration IMU Q
  	Serial.print(cali.acc1);Serial.println(" : Accelerometer calibration IMU Q (front)"); //Accelerometer calibration IMU Q
  	Serial.print(cali.mg1);Serial.println(" : Magnetometer calibration IMU Q (front)"); //Magnetometer calibration IMU Q
  	Serial.print(cali.system2);Serial.println(" : System calibration IMU P (rear)"); //System calibration IMU P
  	Serial.print(cali.gyros2);Serial.println(" : Gyroscope calibration IMU P (rear)"); //Gyroscope calibration IMU P
  	Serial.print(cali.acc2);Serial.println(" : Accelerometer calibration IMU P (rear)"); //Accelerometer calibration IMU P
  	Serial.print(cali.mg2);Serial.println(" : Magnetometer calibration IMU P (rear)"); //Magnetometer calibration IMU P
  	Serial.print(cali.u6_radio*radius);Serial.println(" : Linear forward velocity [m/s]"); //Linear forward speed in m/s
  	Serial.print(cali.u6_d_radio*radius);Serial.println(" : Linear forward acceleration [m/s/s]"); //Linear forward acceleration in m/s
  	Serial.print(cali.u6_radio*radius*3.6);Serial.println(" : Linear forward velocity [km/h]"); //Linear forward speed in km/h
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);                  //Start Serial Monitor.
  delay(100);
  myRadio.begin();                       //Start radio.
  myRadio.setChannel(100);               //Set same channel in transmitter and receiver.
  myRadio.setPALevel(RF24_PA_MIN);       //Radios are close together so low power level.
  myRadio.setDataRate(RF24_250KBPS);     //Data rate low for minimal interference.
  myRadio.openReadingPipe(1, addrs[0]);  //Read from pipe.
  myRadio.startListening();              //Listen for signal from transmitter.
  delay(100);
}

void loop()  {
  currentMillis = millis();

  //Print out monitoring data every interval defined by chosen frequency.
  if(float(currentMillis - previousMillis) >= interval) {
    previousMillis = currentMillis;
    receiveAndPrint();
  }
}
