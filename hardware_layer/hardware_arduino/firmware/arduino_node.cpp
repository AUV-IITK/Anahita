#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <hyperion_msgs/Depth.h>
#include <hyperion_msgs/Pressure.h>

#include "MS5837.h"
#include "ArduinoConfig.h"

Servo servoEast;
Servo servoWest;
Servo servoNorthSway;
Servo servoSouthSway;
Servo servoNorthWestUp;
Servo servoNorthEastUp;
Servo servoSouthWestUp;
Servo servoSouthEastUp;

void TEastCb(const std_msgs::Int32& msg);
void TWestCb(const std_msgs::Int32& msg);
void TNorthCb(const std_msgs::Int32& msg);
void TSouthCb(const std_msgs::Int32& msg);
void TNEUpCb(const std_msgs::Int32& msg);
void TSEUpCb(const std_msgs::Int32& msg);
void TNWUpCb(const std_msgs::Int32& msg);
void TSWUpCb(const std_msgs::Int32& msg);

int ESC_Zero = 1500;
int deadZone = 25;
ros::NodeHandle nh;

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

// declare subscribers
ros::Subscriber<std_msgs::Int32> TEast_PWM_Sub("/pwm/forwardRight", &TEastCb);
ros::Subscriber<std_msgs::Int32> TWest_PWM_Sub("/pwm/forwardLeft", &TWestCb);
ros::Subscriber<std_msgs::Int32> TNorth_PWM_Sub("/pwm/sidewardFront", &TNorthCb);
ros::Subscriber<std_msgs::Int32> TSouth_PWM_Sub("/pwm/sidewardBack", &TSouthCb);
ros::Subscriber<std_msgs::Int32> TNW_PWM_Sub("/pwm/upwardNorthWest", &TNWUpCb);
ros::Subscriber<std_msgs::Int32> TNE_PWM_Sub("/pwm/upwardNorthEast", &TNEUpCb);
ros::Subscriber<std_msgs::Int32> TSE_PWM_Sub("/pwm/upwardSouthEast", &TSEUpCb);
ros::Subscriber<std_msgs::Int32> TSW_PWM_Sub("/pwm/upwardSouthWest", &TSWUpCb);

// function declration to puboish pressure sensor data
void publish_pressure_data();

// declaration of callback functions for all thrusters
// declare publishers
std_msgs::Float32 pressure_msg;
ros::Publisher ps_pressure_pub("/pressure_sensor/pressure", &pressure_msg);
std_msgs::Float32 depth_msg;
ros::Publisher ps_depth_pub("/pressure_sensor/depth", &depth_msg);

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
    nh.subscribe(TEast_PWM_Sub);
    nh.subscribe(TWest_PWM_Sub);
    nh.subscribe(TNorth_PWM_Sub);
    nh.subscribe(TSouth_PWM_Sub);
    nh.subscribe(TNW_PWM_Sub);
    nh.subscribe(TNE_PWM_Sub);
    nh.subscribe(TSW_PWM_Sub);
    nh.subscribe(TSE_PWM_Sub);

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
    
    nh.spinOnce();
}

// function definition for publish_pressure_data
void publish_pressure_data()
{
    pressure_sensor.read();
    pressure_msg.data = pressure_sensor.pressure();

    depth_msg.data = pressure_sensor.depth();

    ps_depth_pub.publish(&depth_msg);
    ps_pressure_pub.publish(&pressure_msg);
}

// definition of callback function
void TEastCb(const std_msgs::Int32& msg)
{
  if(msg.data>0)
    servoEast.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoEast.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoEast.writeMicroseconds(ESC_Zero);
}
void TWestCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoWest.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoWest.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoWest.writeMicroseconds(ESC_Zero);
}
void TNorthCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoNorthSway.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoNorthSway.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoNorthSway.writeMicroseconds(ESC_Zero);
}
void TSouthCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoSouthSway.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoSouthSway.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoSouthSway.writeMicroseconds(ESC_Zero);
}
void TNEUpCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
}
void TSEUpCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
}
void TNWUpCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
}
void TSWUpCb(const std_msgs::Int32& msg)
{
   if(msg.data>0)
    servoSouthWestUp.writeMicroseconds(ESC_Zero + msg.data + deadZone);
  else if(msg.data<0)
    servoSouthWestUp.writeMicroseconds(ESC_Zero + msg.data - deadZone);
  else
    servoSouthWestUp.writeMicroseconds(ESC_Zero);
}
