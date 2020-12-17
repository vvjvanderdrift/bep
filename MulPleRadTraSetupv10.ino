//Transmitter

//Include necessary libraries
#include <SD.h>
#include <SPI.h>  
#include "RF24.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//Define constants & variables.
//Define constants for the measurements & bicycle parameters.
const float freq = 12.5;    // Define the measuring frequency in [Hz].
const float v_min = 1/3.6;    // Set the minimal cycling speed of bicycle in [m/s] for the speedometer to read out.
const float v_max = 50/3.6;   // Set the maximal cycling speed of bicycle in [m/s] for the speedometer to read out.
const float radius = 0.35;    // Set the radius rear wheel in [m].
const float magnets = 3;    // Define amount of magnets on the rear wheel for more accurate measurements.

//Define pins for Arduino setup.
const int chipSelect = 4;     // Set the CS pin of SD card module.
const int buttonPin = 6;    // Set the pin for the button that switches between calibration mode and recording mode.
const int hallSensorPin = 2;    // Set the pin for the output of the hall sensor. This has to be an interrupt pin (2 on Arduino Uno).
const int ledPin = 7;     // Set the pin for the LED that blinks when the hall sensor reads a magnet.

//Define parameters for measurements and recordings.
const String fileNameBegin = "TEST_";    // Define the begin of the name of the file you want to record on the SD card.
const String fileNameEnd = ".CSV";    // Define the end of the name of the file you want to record on the SD card. This also defines the extension, which is .CSV in our case but can also be .TXT or other formats.
int fileNumber = 1;     // Define a starting value to count files.
const int cableAttach = 0;    // Set this to "1" of you have a cable attached to the transmitting Arduino and to "0" if not. This (de)activates the serial connection to save time if not used.
const int cableSave = 0;    // Set this to "1" if you wish to see/save results via the serial monitor. The variable cableAttach has to be set to "1" for this to work.

//Define necessary constants and variables for the calculations. These don't need to be changed.
volatile unsigned long lastturn, time_press;    // Define a variable that saves the time in [ms] of the last turn of the rear wheel to calculate u6 and u6_d.
volatile float u6 = 0;    // Define a variable for the rear wheel angular velocity in [rad/s].
volatile float u6_old = 0;    // Define a variable for the rear wheel angular velocity previous measurement in [rad/s].
volatile float u6_d = 0;    // Define a variable for the rear wheel angular acceleration in [rad/s/s].
File mySensorData;    // Define a file dedicated to save on the SD card.
String fileNameTotal;     // Define a variable that builds the total filename that will be saved. Be aware that the name of the file, excluding the extension, may not be more than 8 characters long in our case.
int buttonPinStatus = 0;    // Define a variable that indicates the state of the buttonPin. This will always be 0 when starting up.
unsigned long currentMillis;    // Define a variable that saves the current time in [ms] for timing the measuring frequency.
unsigned long previousMillis = 0;     // Define a variable that saves the previous time in [ms] for timing the measuring frequency.
float interval = 1000/freq;     // Calculate an interval between measurements with the defined frequency in [ms].
int ledState = 0;     // Define a variable that indicates the state of the LED. 
float dt_min = 2 * PI * 1000 / ((v_max/radius) * magnets);    // Calculate a minimal timestep to calculate standstill in [ms].
float dt_max = 2 * PI * 1000 / ((v_min/radius) * magnets);    // Calculate a maximal timestep in [ms] to keep out noise from the hall sensor.

// Define radio connecting pins and address for transceivers and call Adafruit BNO055 sensors
RF24 myRadio (9, 8); 
byte addrs[][6] = {"T"};
Adafruit_BNO055 bno1 = Adafruit_BNO055(55);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55);


// Define vectors to store the values of the quaternion data, gyroscope readings and linear accelerometer values as measured and calculated by both IMUs.
imu::Quaternion quat1;  // 4x4 bytes
imu::Vector<3> gyro1; // 3x4 bytes
imu::Vector<3> linacc1; // 3x4 bytes
imu::Quaternion quat2;  // 4x4 bytes
imu::Vector<3> gyro2; // 3x4 bytes
imu::Vector<3> linacc2; // 3x4 bytes

// Define a package that contains 20 bytes of calibration data of the IMUs as well as the angular velocity & acceleration (u6 and u6_d). This package will be sent over the radio modules.
struct package0 { 

  uint32_t measurement = 0; // 4 bytes
  uint8_t system1 = 0; // 1 byte
  uint8_t gyros1 = 0; // 1 byte
  uint8_t acc1 = 0; // 1 byte
  uint8_t mg1 = 0; // 1 byte 
  uint8_t system2 = 0; // 1 byte
  uint8_t gyros2 = 0; // 1 byte
  uint8_t acc2 = 0; // 1 byte
  uint8_t mg2 = 0; // 1 byte
  float u6_radio = 0; // 4 bytes
  float u6_d_radio = 0; // 4 bytes
};
typedef struct package0 Package0;
Package0 cali;


// Define a function that selects the right IMU through the multiplexer.
void tcaselect(uint8_t i) {

  if (i > 7) return;
 
  Wire.beginTransmission(0x70);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Define a function that saves the measured data to the SD card.
void sdSave() {

  fileNameTotal = fileNameBegin + String(fileNumber) + fileNameEnd; // 
  mySensorData = SD.open(fileNameTotal, FILE_WRITE);
  if (mySensorData) {
    if (cableAttach == 1 && cableSave == 0) {
      Serial.println("Data printed on SD card");
    }
    
  //Inertial Measurement Unit 1
  mySensorData.print(cali.measurement);mySensorData.print(","); //Measurement number imu 1
  mySensorData.print(cali.system1);mySensorData.print(","); //System calibration imu 1
  mySensorData.print(cali.gyros1);mySensorData.print(","); //Gyroscope calibration imu 1
  mySensorData.print(cali.acc1);mySensorData.print(","); //Accelerometer calibration imu 1
  mySensorData.print(cali.mg1);mySensorData.print(","); //Magnetometer calibration imu 1
  mySensorData.print(quat1.w(), 4);mySensorData.print(","); //Quaternion w imu 1
  mySensorData.print(quat1.x(), 4);mySensorData.print(","); //Quaternion x imu 1
  mySensorData.print(quat1.y(), 4);mySensorData.print(","); //Quaternion y imu 1
  mySensorData.print(quat1.z(), 4);mySensorData.print(","); //Quaternion z imu 1
  mySensorData.print(gyro1.x());mySensorData.print(","); //Gyroscope x imu 1
  mySensorData.print(-gyro1.y());mySensorData.print(","); //Gyroscope y imu 1
  mySensorData.print(-gyro1.z());mySensorData.print(","); //Gyroscope z imu 1
  mySensorData.print(linacc1.x());mySensorData.print(","); //Linear acceleration x imu 1
  mySensorData.print(-linacc1.y());mySensorData.print(","); //Linear acceleration y imu 1
  mySensorData.print(-linacc1.z());mySensorData.print(","); //Linear acceleration z imu 1

  //Inertial Measurement Unit 2
  mySensorData.print(cali.measurement);mySensorData.print(","); //Measurement number imu 2
  mySensorData.print(cali.system2);mySensorData.print(","); //System calibration imu 2
  mySensorData.print(cali.gyros2);mySensorData.print(","); //Gyroscope calibration imu 2
  mySensorData.print(cali.acc2);mySensorData.print(","); //Accelerometer calibration imu 2
  mySensorData.print(cali.mg2);mySensorData.print(","); //Magnetometer calibration imu 2
  mySensorData.print(quat2.w(), 4);mySensorData.print(","); //Quaternion w imu 2
  mySensorData.print(quat2.x(), 4);mySensorData.print(","); //Quaternion x imu 2
  mySensorData.print(quat2.y(), 4);mySensorData.print(","); //Quaternion y imu 2
  mySensorData.print(quat2.z(), 4);mySensorData.print(","); //Quaternion z imu 2
  mySensorData.print(gyro2.x());mySensorData.print(","); //Gyroscope x imu 2
  mySensorData.print(-gyro2.y());mySensorData.print(","); //Gyroscope y imu 2
  mySensorData.print(-gyro2.z());mySensorData.print(","); //Gyroscope z imu 2
  mySensorData.print(linacc2.x());mySensorData.print(","); //Linear acceleration x imu 2
  mySensorData.print(-linacc2.y());mySensorData.print(","); //Linear acceleration y imu 2
  mySensorData.print(-linacc2.z());mySensorData.print(","); //Linear acceleration z imu 2
  
   //Print the angular velocities and acceleration
  mySensorData.print(u6);mySensorData.print(","); //Angular velocity rear wheel
  mySensorData.print(u6_d);mySensorData.print(","); //Angular acceleration rear wheel
  mySensorData.print(u6*radius);mySensorData.print(","); //Linear velocity rear wheel
  mySensorData.print(u6_d*radius);mySensorData.print(","); //Linear velocity rear wheel

  mySensorData.println(currentMillis);

  
  mySensorData.close();
  }
  else{
    if (cableAttach) {
      Serial.println("SD card failed");
      Serial.println();
    }
  }
}

void imuInitialise() {

    // Initialise the 1st IMU.
  tcaselect(2);
  if(!bno1.begin())
  {
    // Error message if IMU doesn't begin.
    if (cableAttach) {
      Serial.println("No BNO055_1 detected ... Check your wiring!");
    }
    while(1);
  }
  
  // Initialise the 2nd IMU.
  tcaselect(6);
  if(!bno2.begin())
  {
    // Error message if IMU doesn't begin
    if (cableAttach) {
      Serial.println("No BNO055_2 detected ... Check your wiring!");
    }
    while(1);
  }
  
}

// Define a function to begin the radio module.
void radioStartup() {

  myRadio.begin();                       // Start radio.
  myRadio.setChannel(100);               // Set same channel in transmitter and receiver.
  myRadio.setPALevel(RF24_PA_MIN);       // Radios are close together so low power level.
  myRadio.setDataRate(RF24_250KBPS);     // Data rate low for minimal interference.
  myRadio.openWritingPipe(addrs[0]);     // Write to pipe.
  delay(100);
}

// Define a function that measures the data from the IMUs.
void imuMeasure() {

  tcaselect(2); // Select the first IMU.
 
  // Get the calibration & measurement data from the first IMU.
  bno1.getCalibration(&cali.system1, &cali.gyros1, &cali.acc1, &cali.mg1); 
  quat1 = bno1.getQuat(); // 16 bytes
  gyro1 = bno1.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  linacc1 = bno1.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  tcaselect(6); // Select the second IMU

  // Get the calibration & measurement data from the second IMU.
  bno2.getCalibration(&cali.system2, &cali.gyros2, &cali.acc2, &cali.mg2);
  quat2 = bno2.getQuat(); // 16 bytes
  gyro2 = bno2.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  linacc2 = bno2.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);  
}

// Define a function that times the hall sensor readings and calculates the rear wheel speed and acceleration with these readings.
void sens() {

  if(millis() - lastturn > dt_min ) { 
    u6 = 2 / magnets * PI / ((float)(millis() - lastturn) / 1000);    //Calculate the rear wheel angular speed u6 by measuring the time it takes in between magnet readings.
    u6_d = (u6 - u6_old) / ((float)(millis() - lastturn) / 1000);    //Calculate the rear wheel angular acceleration u6_d by measuring the time it takes in between magnet readings.
    u6_old = u6;
    lastturn = millis();
    ledState = 1;
  }
}

// Define a function that resets the speedometer values if the time between hall sensor readings becomes too great. This minimal timestep can be set by changing the v_min variable.
void speedometerReset() {

  if ((millis() - lastturn) > dt_max) {
  u6 = 0;
  u6_old = 0;
  u6_d = 0;
  }
}

// Define a function that executes the sdSave or myRadio.write functions, depending on the status of the button.
void buttonRead() {

  if (digitalRead(buttonPin) == HIGH) {
    if (cableAttach == 1 && cableSave ==0) {
      Serial.print(cali.measurement);
      Serial.print(" Buttonpin On, Radio Transmission Off & SD Card On, filenumber: "); Serial.println(fileNumber);

  //Serial.print(float(currentMillis)/1000);Serial.println(" : Timestamp since start in seconds");
  Serial.println();
      
      Serial.println();
    }
  sdSave();
  buttonPinStatus = 1;
  }

  if (digitalRead(buttonPin) == LOW){
    if (cableAttach == 1 && cableSave == 0) {
      Serial.print("Time since start [ms]: ");
      Serial.print(currentMillis);
      Serial.print("; Measurement number: ");
      Serial.print(cali.measurement);
      Serial.println("; Buttonpin Off, Radio Transmission On & SD Card Off");
 
  //Serial.print(float(currentMillis)/1000);Serial.println(" : Timestamp since start in seconds");
  Serial.println();
      
      Serial.println();
    }
    // Copy the current u6 and u6_d to files transmittable by radio to monitor the current speed of the bicycle.
    cali.u6_radio = u6;
    cali.u6_d_radio = u6_d;
  
    myRadio.write(&cali, sizeof(cali));     // Send the data package cali to the receiver.

    // With the following if-statement, we count the number of recordings taken (how many times the button was switched).
    if (buttonPinStatus == 1) {
      fileNumber = fileNumber + 1;
    }

  buttonPinStatus = 0;
  }  
}

// Define a function that prints the data via the serial monitor in case it needs to be recorded via the USB cable. 
void printViaCable() {
  
  Serial.print(cali.measurement);Serial.print(","); //Measurement number imu 1
  Serial.print(cali.system1);Serial.print(","); //System calibration imu 1
  Serial.print(cali.gyros1);Serial.print(","); //Gyroscope calibration imu 1
  Serial.print(cali.acc1);Serial.print(","); //Accelerometer calibration imu 1
  Serial.print(cali.mg1);Serial.print(","); //Magnetometer calibration imu 1
  Serial.print(quat1.w(), 4);Serial.print(","); //Quaternion w imu 1
  Serial.print(quat1.x(), 4);Serial.print(","); //Quaternion x imu 1
  Serial.print(quat1.y(), 4);Serial.print(","); //Quaternion y imu 1
  Serial.print(quat1.z(), 4);Serial.print(","); //Quaternion z imu 1
  Serial.print(gyro1.x());Serial.print(","); //Gyroscope x imu 1
  Serial.print(-gyro1.y());Serial.print(","); //Gyroscope y imu 1
  Serial.print(-gyro1.z());Serial.print(","); //Gyroscope z imu 1
  Serial.print(linacc1.x());Serial.print(","); //Linear acceleration x imu 1
  Serial.print(-linacc1.y());Serial.print(","); //Linear acceleration y imu 1
  Serial.print(-linacc1.z());Serial.print(","); //Linear acceleration z imu 1

  Serial.print(cali.measurement);Serial.print(","); //Measurement number imu 2
  Serial.print(cali.system2);Serial.print(","); //System calibration imu 2
  Serial.print(cali.gyros2);Serial.print(","); //Gyroscope calibration imu 2
  Serial.print(cali.acc2);Serial.print(","); //Accelerometer calibration imu 2
  Serial.print(cali.mg2);Serial.print(","); //Magnetometer calibration imu 2
  Serial.print(quat2.w(), 4);Serial.print(","); //Quaternion w imu 2
  Serial.print(quat2.x(), 4);Serial.print(","); //Quaternion x imu 2
  Serial.print(quat2.y(), 4);Serial.print(","); //Quaternion y imu 2
  Serial.print(quat2.z(), 4);Serial.print(","); //Quaternion z imu 2
  Serial.print(gyro2.x());Serial.print(","); //Gyroscope x imu 2
  Serial.print(-gyro2.y());Serial.print(","); //Gyroscope y imu 2
  Serial.print(-gyro2.z());Serial.print(","); //Gyroscope z imu 2
  Serial.print(linacc2.x());Serial.print(","); //Linear acceleration x imu 2
  Serial.print(-linacc2.y());Serial.print(","); //Linear acceleration y imu 2
  Serial.print(-linacc2.z());Serial.print(","); //Linear acceleration z imu 2
  
  //Print the angular velocities and acceleration.
  Serial.print(u6);Serial.print(","); //Angular velocity rear wheel
  Serial.print(u6_d);Serial.print(","); //Angular acceleration rear wheel
  Serial.print(u6*radius);Serial.print(","); //Linear velocity rear wheel
  Serial.print(u6_d*radius);Serial.print(","); //Linear velocity rear wheel
  Serial.println(currentMillis);
}

//Define a function that blinks the LED if the hall sensor senses a magnet.
void ledBlink() {

  if (ledState == 0) {        
    digitalWrite(ledPin, LOW);  
  } 
  else if (ledState == 1) {
    digitalWrite(ledPin, HIGH);
    ledState = 0;
  }
}

// Startup Arduino.
void setup() {

  //Start serial sonitor if USB cable is attached.
  if (cableAttach) {
    Serial.begin(115200);
  }

  //Initiate the Wire library and join the I2C bus for communication with the multiplexer & IMUs. 
  Wire.begin();

  //Declare LED pin.
  pinMode(ledPin, OUTPUT);

  //Declare interrupt for sens function to determine u6 en u6_d.
  attachInterrupt(digitalPinToInterrupt(hallSensorPin),sens,FALLING);
     
  //Declare SD pins.
  pinMode(10, OUTPUT);
  SD.begin(4);

  //Declare button pin.
  pinMode(buttonPin, INPUT);
    
  //Initialise IMUs.
  imuInitialise();

  //Startup radio.
  radioStartup();
}

//Start the main loop function to execute main commands.
void loop() {
  currentMillis = millis();

  
  //Run the main commands in an interval determined by the declared measurement frequency.
  if ((currentMillis - previousMillis) >= long(interval)) {

    Serial.println();
    Serial.println(interval);
    Serial.println(currentMillis - previousMillis);
    
    previousMillis = currentMillis;

    ledBlink();
    
    imuMeasure();

    speedometerReset();

    //Read out button status decide whether the system is in calibrate modus or in record modus and act accordingly.
    buttonRead();

    //Send results via USB serial connection if wanted.
    if (cableSave) {
      printViaCable();
    }

    //Increase measurement counter
    cali.measurement = cali.measurement + 1;
  }  
  



}
