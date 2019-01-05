#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <hyperion_msgs/Depth.h>
#include <hyperion_msgs/Pressure.h>
#include <hyperion_msgs/Thrust.h>

#include <MS5837.h>

#define MDpin1 25
#define MDpin2 24

#define servoEastPin 40       // pin definitions for forward thrusters
#define servoWestPin 43

#define servoNorthSwayPin 31   // pin definitions for sideward thrusters
#define servoSouthSwayPin 30

#define servoNorthWestUpPin 28// pin definitions for upward thrusters
#define servoSouthWestUpPin 32
#define servoNorthEastUpPin 27 
#define servoSouthEastUpPin 33

#define permanentGround1 29
#define permanentGround2 26
#define permanentGround3 41
#define permanentGround4 42

#define analogPinPressureSensor A0      //pin definition for depth sensor

Servo servoEast;
Servo servoWest;
Servo servoNorthSway;
Servo servoSouthSway;
Servo servoNorthWestUp;
Servo servoNorthEastUp;
Servo servoSouthWestUp;
Servo servoSouthEastUp;

void PWMCb(const hyperion_msgs::Thrust& msg_);
void TEast(const int data);
void TWest(const int data);
void TNorth(const int data);
void TSouth(const int data);
void TNEUp(const int data);
void TNWUp(const int data);
void TSWEUp(const int data);
void TSWIUp(const int data);

void pressureCb(const std_msgs::Bool& _msg);

int ESC_Zero = 1500;
int deadZone = 25;
ros::NodeHandle nh;

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

// declare subscribers
ros::Subscriber<hyperion_msgs::Thrust> PWM_Sub("/pwm", &PWMCb);

// function declration to puboish pressure sensor data
void publish_pressure_data();

// declaration of callback functions for all thrusters
// declare publishers
std_msgs::Float32 pressure_msg;
ros::Publisher ps_pressure_pub("/pressure_sensor/pressure", &pressure_msg);
std_msgs::Float32 depth_msg;
ros::Publisher ps_depth_pub("/anahita/z_coordinate", &depth_msg);
bool enable_pressure = false;

void setup()
{
    servoEast.attach(servoEastPin);
    servoEast.writeMicroseconds(ESC_Zero);
    servoNorthSway.attach(servoNorthSwayPin);
    servoNorthSway.writeMicroseconds(ESC_Zero);
    servoWest.attach(servoWestPin);
    servoWest.writeMicroseconds(ESC_Zero);
    servoSouthSway.attach(servoSouthSwayPin);
    servoSouthSway.writeMicroseconds(ESC_Zero);
    
    servoNorthWestUp.attach(servoNorthWestUpPin);
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
    servoNorthEastUp.attach(servoNorthEastUpPin);
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
    servoSouthWestUp.attach(servoSouthWestUpPin);
    servoSouthWestUp.writeMicroseconds(ESC_Zero);
    servoSouthEastUp.attach(servoSouthEastUpPin);
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
    
    pinMode(permanentGround1, OUTPUT);
    pinMode(permanentGround2, OUTPUT);
    pinMode(permanentGround3, OUTPUT);
    pinMode(permanentGround4, OUTPUT);

    digitalWrite(permanentGround1, LOW);
    digitalWrite(permanentGround2, LOW);
    digitalWrite(permanentGround3, LOW);
    digitalWrite(permanentGround4, LOW);
    
    delay(7000);
    Serial.print("Successfully started servos");    
    
    // setting up pressure sensor
    Wire.begin();
    // We can't continue with the rest of the program unless we can initialize the sensor
    pressure_sensor.init();
    pressure_sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)*/
    
    //initialize ROS node
    nh.initNode();
    nh.getHardware()->setBaud(57600);

    //start ros_node
    nh.subscribe(PWM_Sub);

    // publisher
    nh.advertise(ps_pressure_pub);
    nh.advertise(ps_depth_pub);
    
    while (!nh.connected())
    {
         nh.spinOnce();
    }
    nh.loginfo("Anahita CONNECTED");
}

void loop()
{
     static unsigned long prev_pressure_time = 0;

    //this block publishes the pressure sensor data based on defined rate
    if ((millis() - prev_pressure_time) >= (1000 / PRESSURE_PUBLISH_RATE))
    {
        publish_pressure_data();
        prev_pressure_time = millis();
    }
    
    nh.loginfo("Data being recieved");
    delay(10);

    // nh.getParam("/enable_pressure", &enable_pressure, 1);
    
    nh.spinOnce();
}

// function definition for publish_pressure_data
void publish_pressure_data()
{
    pressure_sensor.read();
    pressure_msg.data = pressure_sensor.pressure();

    /** Depth returned in meters (valid for operation in incompressible
    *  liquids only. Uses density that is set for fresh or seawater.
    */
    depth_msg.data = -100*pressure_sensor.depth(); //convert to centimeters

    if (enable_pressure) {
      ps_depth_pub.publish(&depth_msg);
    }
    ps_pressure_pub.publish(&pressure_msg);
}

void TEast(const int data)
{
  if(data>0)
    servoEast.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoEast.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoEast.writeMicroseconds(ESC_Zero);
}
void TWest(const int data)
{
  if(data>0)
    servoWest.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoWest.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoWest.writeMicroseconds(ESC_Zero);
}
void TNorth(const int data)
{
  if(data>0)
    servoNorthSway.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthSway.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthSway.writeMicroseconds(ESC_Zero);
}
void TSouth(const int data)
{
  if(data>0)
    servoSouthSway.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthSway.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthSway.writeMicroseconds(ESC_Zero);
}
void TNEUp(const int data)
{
  if(data>0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
}
void TNWUp(const int data)
{
  if(data>0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
}
void TSEUp(const int data)
{
  if(data>0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
}
void TSWUp(const int data)
{
  if(data>0)
    servoSouthWestUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthWestUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthWestUp.writeMicroseconds(ESC_Zero);
}

void PWMCb(const hyperion_msgs::Thrust& msg_)
{
    nh.loginfo("Inside PWM Callback");
    TEast(msg_.forward_right);
    TWest(msg_.forward_left);
    TNorth(msg_.sideward_front);
    TSouth(msg_.sideward_back);
    TNEUp(msg_.upward_north_east);
    TNWUp(msg_.upward_north_west);
    TSEUp(msg_.upward_south_east);
    TSWUp(msg_.upward_south_west); 
}

void pressureCb(const std_msgs::Bool& _msg) {
  enable_pressure = _msg.data;
}
