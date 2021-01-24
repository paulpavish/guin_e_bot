/*
  //   Title : GuiN-E NMCU Ver. 1.0.0
  //   Author : Paul Pavish
  //   Website : www.basicsexplained.com/creator
  //   YT Channel : https://www.youtube.com/channel/UCavN7aolUmBbfcKbDbydvMA
  //   LinkedIn : https://www.linkedin.com/in/paulpavish/
  //
  //   Kindly attribute to Author on any marvelous project you come up with using this piece of code.
  //   Also show your support at my Youtube Channel. Thankyou.
  //
  //   The sketch connects to 'rosserial_python' TCP socket through the 'guin_e_turtle.launch' ROS launch file.
  //   In Linux Terminal : 'roslaunch guin_e_turtle guin_e_turtle.launch' to launch the ros nodes
  //   This Sketch can control the GuiN-E Bot V1.0 base motors by communicating with an Arduino UNO through I2C Protocol.
  //   The NodeMCU acts as communiication brigde between ROS Node (TDP Socket) and Arduino UNO (I2C Slave).
  //   The Data received will be in the form of ROS std_msgs/ByteMultiArray Message (byte[] data)type.
  //   The Data sent will be in the form of byte array.
  //
  //  This Sketch is purposefully constructed for easy future updation for upcoming GuiN-E Bot versions.

*/


//Including Libraries
#include<ESP8266WiFi.h>
#include<Wire.h>
#include<ros.h>
#include<std_msgs/UInt16MultiArray.h>

//Defining I2C Pins
#define SDA_PIN D2
#define SCL_PIN D1

//I2C Slave Address in HEX
const int16_t I2C_SLAVE = 0x08;

//WiFi SSID & Password
const char* ssid     = "p@v!5h_pr!5h@";
const char* password = "2905&1910";

//IP address of ROS Machine (Find using Find_IP.py in linux machine)
IPAddress server(192, 168, 1, 6);

//Setting the rosserial socket server port
const uint16_t serverPort = 11411; //DO NOT CHANGE

//Wire Buffer
int buff[5];

//Creating ROS Node object
ros::NodeHandle GuiNE;

//Subscribe Callback function
void callback( const std_msgs::UInt16MultiArray& uint16Arr) {
  //Begin Wire Transmission
  Wire.beginTransmission(I2C_SLAVE);
  for (int i = 0; i <= sizeof(uint16Arr.data); i++) {
    buff[i] = uint16Arr.data[i];
    Wire.write(buff[i]);
    Serial.print(buff[i]);
  }
  Serial.println();
  Wire.endTransmission();
}

//Make a ROS Subscriber Object
ros::Subscriber<std_msgs::UInt16MultiArray> sub("GuiNE_Bot/Motor", &callback );

void setup() {
  //Serial Initiation
  Serial.begin(115200);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //Connecting to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  //Initializing I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  //Initializing ROS Node
  GuiNE.getHardware()->setConnection(server, serverPort);
  GuiNE.initNode();
  GuiNE.subscribe(sub);
}

void loop() {
  //Keeping the ROS Node Subscribing
  GuiNE.spinOnce();
  delay(1);
}
