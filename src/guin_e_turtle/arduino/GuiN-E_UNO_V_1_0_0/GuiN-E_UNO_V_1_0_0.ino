/*
  //   Title : GuiN-E UNO Ver. 1.0.0
  //   Author : Paul Pavish
  //   Website : www.basicsexplained.com/creator
  //   YT Channel : https://www.youtube.com/channel/UCavN7aolUmBbfcKbDbydvMA
  //   LinkedIn : https://www.linkedin.com/in/paulpavish/
  //
  //   Kindly attribute to Author on any marvelous project you come up with using this piece of code.
  //   Also show your support on the Author's Youtube Channel. Thankyou.
  //
  //   Note : Before Uploading this sketch interface the motor driver and calibrate the connection
  //          using GuiN-E UNO Motor Calibration V1.0 example from MotorDriver Library by Paul Pavish
  //
  //   This Sketch can control the GuiN-E Bot V1.0 base motors by communicating with a NodeMCU 1.0 (ESP-12E) through I2C Protocol.
  //   The Uno Acts as an I2C Slave. SDA_PIN = A4; SCL_PIN = A5
  //   The array of data bytes received will be control values for the Base Motor Driver.
  //
  //  This Sketch is purposefully constructed for easy future updation for upcoming GuiN-E Bot versions.

*/

//Including Libraries
#include<MotorDriver.h>
#include<Wire.h>

//DO NOT CHANGE THESE PINS(Calibrated using MotorDriver Library Example)
#define SpeedPin 3
#define LeftCtrl_A  4
#define LeftCtrl_B  5
#define RightCtrl_A 6
#define RightCtrl_B 7

//Creating Base Object for MotorDriver
MotorDriver Base(SpeedPin, LeftCtrl_A, LeftCtrl_B, RightCtrl_A, RightCtrl_B);

void setup() {
  // Initialize I2C Slave on Address 8
  Wire.begin(8);
  Wire.onReceive(ReceiveEvent);

  //Initialize Serial
  
    Serial.begin(115200);
    Serial.println(F("Initialized. Waiting for I2C Master."));
  
}

//I2C onReceive Function
void ReceiveEvent(int howMany) {
  byte m_c[howMany];
  int i = 0;
  while (Wire.available()) {
    m_c[i] = Wire.read();
    i++;
  }
  //Serial Print Received Data
  
    for ( i = 0; i < howMany; i++) {
    Serial.print(m_c[i]);
    }
    Serial.println();
  

  //Control Base
  Base.Control(m_c[0], m_c[1], m_c[2], m_c[3], m_c[4]);
}

void loop() {
  //delay(100);
}
