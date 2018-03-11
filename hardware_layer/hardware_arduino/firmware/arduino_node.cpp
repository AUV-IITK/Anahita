#include "ros.h"
#include <Arduino.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <Wire.h>
#include "../include/MS5837.h"
#include "../include/arduino_thrust.h"

#define pwmPinWest 3
#define pwmPinEast 2
#define directionPinEast1 30
#define directionPinEast2 31
#define directionPinWest1 32
#define directionPinWest2 33

#define pwmPinNorthSway 5
#define pwmPinSouthSway 4
#define directionPinSouthSway1 27
#define directionPinSouthSway2 26
#define directionPinNorthSway1 29
#define directionPinNorthSway2 287

#define pwmPinNorthUp 6
#define pwmPinSouthUp 7
#define directionPinNorthUp1 24
#define directionPinNorthUp2 25
#define directionPinSouthUp1 22
#define directionPinSouthUp2 23

#define analogPinPressureSensor A0

MS5837 sensor;

arduino_thrust AEAST,AWEST,ANORTHUP,ASOUTHUP,ANORTHSWAY,ASOUTHSWAY;

float last_pressure_sensor_value, pressure_sensor_value;
std_msgs::Float64 voltage;
ros::NodeHandle nh;

void TEastCb(const std_msgs::Int32 msg)
{
    int pwm = msg.data;
    pwm = abs(pwm);
    bool isForward = true;
    if(msg.data <= 0)
        isForward = false;
    if(isForward)
        AEAST.ON(255-pwm, HIGH, LOW);
    else
        AEAST.ON(255-pwm, LOW, HIGH);
}


void TWestCb(const std_msgs::Int32 msg)
{
    int pwm = msg.data;
    pwm = abs(pwm);
    bool isForward = true;
    if(msg.data <= 0)
        isForward = false;
    if(isForward)
        AWEST.ON(255-pwm, HIGH, LOW);
    else
        AWEST.ON(255-pwm, LOW, HIGH);
}

void TNorthSwayCb(const std_msgs::Int32 msg)
{
    int pwm = msg.data;
    pwm = abs(pwm);
    bool isSideward = true;
    if(msg.data <= 0)
        isSideward = false;
    if(isSideward)
        ANORTHSWAY.ON(255-pwm, HIGH, LOW);
    else
        ANORTHSWAY.ON(255-pwm, LOW, HIGH);
}

void TSouthSwayCb(const std_msgs::Int32 msg)
{
    int pwm = msg.data;
    pwm = abs(pwm);
    bool isSideward = true;
    if(msg.data <= 0)
        isSideward = false;
    if(isSideward)
        ASOUTHSWAY.ON(255-pwm, HIGH, LOW);
    else
        ASOUTHSWAY.ON(255-pwm, LOW, HIGH);
}

void TNorthUpCb(const std_msgs::Int32 msg)
{
    int pwm = msg.data;
    pwm = abs(pwm);
    bool isUpward = true;
    if(msg.data <= 0)
        isUpward = false;
    if(isUpward)
        ANORTHUP.ON(255-pwm, HIGH, LOW);
    else
        ANORTHUP.ON(255-pwm, LOW, HIGH);
}

void TSouthUpCb(const std_msgs::Int32 msg)
{
    int pwm = msg.data;
    pwm = abs(pwm);
    bool isUpward = true;
    if(msg.data <= 0)
        isUpward = false;
    if(isUpward)
        ASOUTHUP.ON(255-pwm, HIGH, LOW);
    else
        ASOUTHUP.ON(255-pwm, LOW, HIGH);
}

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
    
    nh.subscribe(subPwmEast);
    nh.subscribe(subPwmWest);
    nh.subscribe(subPwmNorthSway);
    nh.subscribe(subPwmSouthSway);
    nh.subscribe(subPwmNorthUp);
    nh.subscribe(subPwmSouthUp);
    nh.advertise(ps_voltage);
    
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
