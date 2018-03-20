/*#if (ARDUINO >= 100)
    #include <Arduino.h>
#else
    #include <WProgram.h>
#endif*/

#include "ros.h"
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <Wire.h>
#include "../include/MS5837.h"
#include "../include/ArduinoThrust.h"
#include "../include/ArduinoConfig.h"

MS5837 sensor;

ArduinoThrust AEAST; 
ArduinoThrust AWEST;
ArduinoThrust ANORTHUP;
ArduinoThrust ASOUTHUP;
ArduinoThrust ANORTHSWAY;
ArduinoThrust ASOUTHSWAY;

float last_pressure_sensor_value, pressure_sensor_value;
std_msgs::Float64 voltage;
ros::NodeHandle nh;

void TEastCb(const std_msgs::Int32& msg);
void TWestCb(const std_msgs::Int32& msg);
void TNorthSwayCb(const std_msgs::Int32& msg);
void TSouthSwayCb(const std_msgs::Int32& msg);
void TNorthUpCb(const std_msgs::Int32& msg);
void TSouthUpCb(const std_msgs::Int32& msg);

ros::Subscriber<std_msgs::Int32> subPwmEast("/ard/east", &TEastCb);
ros::Subscriber<std_msgs::Int32> subPwmWest("/ard/west", &TWestCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthSway("/ard/northsway", &TNorthSwayCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthSway("/ard/southsway", &TSouthSwayCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthUp("/ard/northup", &TNorthUpCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthUp("/ard/southup", &TSouthUpCb);
ros::Publisher ps_voltage("/varun/sensors/pressure_sensor/depth", &voltage);

void setup()
{
    nh.initNode();
    Wire.begin();
    
    sensor.init();
    
    sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)
    
    //setting the pins to output mode
    AEAST.setPins(pwmPinEast,directionPinEast1,directionPinEast2);
    AWEST.setPins(pwmPinWest,directionPinWest1,directionPinWest2);
    ANORTHSWAY.setPins(pwmPinNorthSway,directionPinNorthSway1,directionPinNorthSway2);
    ASOUTHSWAY.setPins(pwmPinSouthSway,directionPinSouthSway1,directionPinSouthSway2);
    ANORTHUP.setPins(pwmPinNorthUp,directionPinNorthUp1,directionPinNorthUp2);
    ASOUTHUP.setPins(pwmPinSouthUp,directionPinSouthUp1,directionPinSouthUp2);
    
    nh.getHardware()->setBaud(57600);

    nh.subscribe(subPwmEast);
    nh.subscribe(subPwmWest);
    nh.subscribe(subPwmNorthSway);
    nh.subscribe(subPwmSouthSway);
    nh.subscribe(subPwmNorthUp);
    nh.subscribe(subPwmSouthUp);
    nh.advertise(ps_voltage);
    
    while (!nh.connected())
    {
        nh.spinOnce();
    }
    Serial.begin(57600);
    
    std_msgs::Int32 v;
    v.data = 0;
    TEastCb(v);
    TWestCb(v);
    TNorthSwayCb(v);
    TSouthSwayCb(v);
    TNorthUpCb(v);
    TSouthUpCb(v);
    
    sensor.read();
    last_pressure_sensor_value = -(sensor.depth() * 100);
}

void loop()
{
    sensor.read();
    // voltage.data made -ve because pressure sensor data should increase going up
    pressure_sensor_value = -(sensor.depth() * 100);
    // to avoid random high values
    if (abs(last_pressure_sensor_value - pressure_sensor_value) < 100)
    {
        voltage.data = 0.7 * pressure_sensor_value + 0.3 * last_pressure_sensor_value;
        ps_voltage.publish(&voltage);
        last_pressure_sensor_value = pressure_sensor_value;
    }
    delay(200);
    nh.spinOnce();
}


void TEastCb(const std_msgs::Int32& msg)
{
   AEAST.ON(msg.data);
}


void TWestCb(const std_msgs::Int32& msg)
{
    AWEST.ON(msg.data);
}

void TNorthSwayCb(const std_msgs::Int32& msg)
{
   ANORTHSWAY.ON(msg.data);
}

void TSouthSwayCb(const std_msgs::Int32& msg)
{
    ASOUTHSWAY.ON(msg.data);
}

void TNorthUpCb(const std_msgs::Int32& msg)
{
   ANORTHUP.ON(msg.data);
}

void TSouthUpCb(const std_msgs::Int32& msg)
{
    ASOUTHUP.ON(msg.data);
}
