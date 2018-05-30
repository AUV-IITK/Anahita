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
#include "../include/ESC.h"
//#include "..include/Servo.h"

//declaration of pressure sensor
MS5837 sensor;

//declaration of ESCs of T200 thrusters through the constructors
ESC TEAST(servoPinEast,1100,1900,1500);
ESC TWEST(servoPinWest,1100,1900,1500);
ESC TNORTHSWAY(servoPinNorthSway,1100,1900,1500);
ESC TSOUTHSWAY(servoPinSouthSway,1100,1900,1500);

//declaration of ohther upward thrusters
ArduinoThrust ANWUP;
ArduinoThrust ASWUP;
ArduinoThrust ANEUP;
ArduinoThrust ASEUP;

float last_pressure_sensor_value, pressure_sensor_value;
std_msgs::Float64 voltage;
ros::NodeHandle nh;

//declaration of callback functions for all thrusters
void TEastCb(const std_msgs::Int32& msg);
void TWestCb(const std_msgs::Int32& msg);
void TNorthSwayCb(const std_msgs::Int32& msg);
void TSouthSwayCb(const std_msgs::Int32& msg);
void TNEUpCb(const std_msgs::Int32& msg);
void TNWUpCb(const std_msgs::Int32& msg);
void TSEUpCb(const std_msgs::Int32& msg);
void TSWUpCb(const std_msgs::Int32& msg);

//declaration of all subscribers and publishers
ros::Subscriber<std_msgs::Int32> subPwmEast("/ard/east", &TEastCb);
ros::Subscriber<std_msgs::Int32> subPwmWest("/ard/west", &TWestCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthSway("/ard/northsway", &TNorthSwayCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthSway("/ard/southsway", &TSouthSwayCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthEastUp("/ard/neup", &TNEUpCb);
ros::Subscriber<std_msgs::Int32> subPwmNorthWestUp("/ard/nwup", &TNWUpCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthEastUp("/ard/seup", &TSEUpCb);
ros::Subscriber<std_msgs::Int32> subPwmSouthWestUp("/ard/swup", &TSWUpCb);
ros::Publisher ps_voltage("/varun/sensors/pressure_sensor/depth", &voltage);

void setup()
{
    nh.initNode();
    Wire.begin();
    
    sensor.init();
    
    sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)
    
    //setting the pins to output mode
    ANEUP.setPins(pwmPinNorthEastUp,directionPinNorthEastUp1,directionPinNorthEastUp2);
    ASEUP.setPins(pwmPinSouthEastUp,directionPinSouthEastUp1,directionPinSouthEastUp2);
    ANWUP.setPins(pwmPinNorthWestUp,directionPinNorthWestUp1,directionPinNorthWestUp2);
    ASWUP.setPins(pwmPinSouthWestUp,directionPinSouthWestUp1,directionPinSouthWestUp2);
    
    //calibrating the ESC modules and LED switching shows, when the process is complete
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    TEAST.calibrate();
    TWEST.calibrate();
    TNORTHSWAY.calibrate();
    TSOUTHSWAY.calibrate();
    digitalWrite(LED_BUILTIN, LOW);
    
    nh.getHardware()->setBaud(57600);

    nh.subscribe(subPwmEast);
    nh.subscribe(subPwmWest);
    nh.subscribe(subPwmNorthSway);
    nh.subscribe(subPwmSouthSway);
    nh.subscribe(subPwmNorthEastUp);
    nh.subscribe(subPwmNorthWestUp);
    nh.subscribe(subPwmSouthEastUp);
    nh.subscribe(subPwmSouthWestUp);
    nh.advertise(ps_voltage);
    
    while (!nh.connected())
    {
        nh.spinOnce();
    }
    Serial.begin(57600);
    
    //sending initial message to all thrusters to stop
    std_msgs::Int32 v;
    v.data = 1500;
    TEastCb(v);
    TWestCb(v);
    TNorthSwayCb(v);
    TSouthSwayCb(v);
    
    v.data = 0;
    TNEUpCb(v);
    TNWUpCb(v);
    TSEUpCb(v);
    TSWUpCb(v);
    
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

//definition of callback functions
void TEastCb(const std_msgs::Int32& msg)
{
    TEAST.speed(msg.data);
}


void TWestCb(const std_msgs::Int32& msg)
{
    TWEST.speed(msg.data);
}

void TNorthSwayCb(const std_msgs::Int32& msg)
{
   TNORTHSWAY.speed(msg.data);
}

void TSouthSwayCb(const std_msgs::Int32& msg)
{
    TSOUTHSWAY.speed(msg.data);
}

void TNEUpCb(const std_msgs::Int32& msg)
{
    ANEUP.ON(msg.data);
}

void TSEUpCb(const std_msgs::Int32& msg)
{
    ASEUP.ON(msg.data);
}

void TNWUpCb(const std_msgs::Int32& msg)
{
    ANWUP.ON(msg.data);
}

void TSWUpCb(const std_msgs::Int32& msg)
{
    ASWUP.ON(msg.data);
}

