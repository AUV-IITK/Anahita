#include <Arduino.h>
#include <Wire.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
// #include <hyperion_msgs/Depth.h>
// #include <hyperion_msgs/Pressure.h>

#include "../include/MS5837.h"
#include "../include/Thruster.h"
#include "../include/ArduinoConfig.h"
#include "../include/ESC.h"

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

/*
ROBOT ORIENTATION
                       NORTH
                    -----------
       NORTH-WEST-UP|         |NORTH-EAST-UP
                    |         |
                    |    -    |
               WEST |    -    | EAST
                    |    -    |
                    |         |
       SOUTH-WEST-UP|         |SOUTH-EAST-UP
                    -----------  
                       SOUTH
*/

// declaration of ESC objects for T200 thrusters 
ESC T_EAST(servoPinEast,1100,1900,1500);
ESC T_WEST(servoPinWest,1100,1900,1500);
ESC T_NORTH(servoPinNorthSway,1100,1900,1500);
ESC T_SOUTH(servoPinSouthSway,1100,1900,1500);

// for upward thrusters
Thruster T_NORTH_EAST_UP(pwmPinNorthEastUp, directionPinNorthEastUp1, directionPinNorthEastUp2);
Thruster T_NORTH_WEST_UP(pwmPinNorthWestUp, directionPinNorthWestUp1, directionPinNorthWestUp2);
Thruster T_SOUTH_WEST_UP(pwmPinSouthEastUp, directionPinSouthEastUp1, directionPinSouthEastUp2);
Thruster T_SOUTH_EAST_UP(pwmPinSouthEastUp, directionPinSouthEastUp1, directionPinSouthEastUp2);

// function declration to puboish pressure sensor data
void publish_pressure_data();

// declaration of callback functions for all thrusters
void TEastCb(const std_msgs::Int32& msg);
void TWestCb(const std_msgs::Int32& msg);
void TNorthCb(const std_msgs::Int32& msg);
void TSouthCb(const std_msgs::Int32& msg);
void TNEUpCb(const std_msgs::Int32& msg);
void TNWUpCb(const std_msgs::Int32& msg);
void TSEUpCb(const std_msgs::Int32& msg);
void TSWUpCb(const std_msgs::Int32& msg);

// defining rosnode handle
ros::NodeHandle nh;

// declare subscribers
ros::Subscriber<std_msgs::Int32> TEast_PWM_Sub("/thruster/east/pwm", &TEastCb);
ros::Subscriber<std_msgs::Int32> TWest_PWM_Sub("/thruster/west/pwm", &TWestCb);
ros::Subscriber<std_msgs::Int32> TNorth_PWM_Sub("/thruster/north/pwm", &TNorthCb);
ros::Subscriber<std_msgs::Int32> TSouth_PWM_Sub("/thruster/south/pwm", &TSouthCb);
ros::Subscriber<std_msgs::Int32> TNW_PWM_Sub("/thruster/north-west/pwm", &TNWUpCb);
ros::Subscriber<std_msgs::Int32> TNE_PWM_Sub("/thruster/north-east/pwm", &TNEUpCb);
ros::Subscriber<std_msgs::Int32> TSE_PWM_Sub("/thruster/south-east/pwm", &TSEUpCb);
ros::Subscriber<std_msgs::Int32> TSW_PWM_Sub("/thruster/south-west/pwm", &TSWUpCb);

// declare publishers
std_msgs::Float32 pressure_msg;
ros::Publisher ps_pressure_pub("/pressure_sensor/pressure", &pressure_msg);
// hyperion_msgs::Pressure depth_msg;
// ros::Publisher ps_depth_pub("/pressure_sensor/depth", &depth_msg);

void setup()
{
    //setting up thruster pins
    T_NORTH_EAST_UP.setup();
    T_NORTH_WEST_UP.setup();
    T_SOUTH_EAST_UP.setup();
    T_SOUTH_WEST_UP.setup();

    // calibrating ESCs
    T_EAST.calibrate();
    T_WEST.calibrate();
    T_NORTH.calibrate();
    T_SOUTH.calibrate();
 
    // setting up pressure sensor
    Wire.begin();
    // We can't continue with the rest of the program unless we can initialize the sensor
    while (!pressure_sensor.init())
    {
        nh.loginfo("Init failed!");
        nh.loginfo("Are SDA/SCL connected correctly?");
        nh.loginfo("Blue Robotics Bar30: White=SDA, Green=SCL");
        nh.loginfo("\n\n");
        delay(5000);
    }
    pressure_sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)

    // initialize ROS node
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    // subscribers
    nh.subscribe(TEast_PWM_Sub);
    nh.subscribe(TWest_PWM_Sub);
    nh.subscribe(TNorth_PWM_Sub);
    nh.subscribe(TSouth_PWM_Sub);
    nh.subscribe(TNW_PWM_Sub);
    nh.subscribe(TNE_PWM_Sub);
    nh.subscribe(TSW_PWM_Sub);
    nh.subscribe(TSE_PWM_Sub);

    // sending initial message to all thrusters to stop
    std_msgs::Int32 v;
    v.data = 1500;
    TEastCb(v);
    TWestCb(v);
    TNorthCb(v);
    TSouthCb(v);
    
    v.data = 0;
    TNEUpCb(v);
    TNWUpCb(v);
    TSEUpCb(v);
    TSWUpCb(v);

    // publisher
    nh.advertise(ps_pressure_pub);
    // nh.advertise(ps_depth_pub);

    while (!nh.connected())
    {
        nh.spinOnce();
    }
    nh.loginfo("Hyperion CONNECTED");
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

    nh.spinOnce();
}

// function definition for publish_pressure_data
void publish_pressure_data()
{
    pressure_sensor.read();
    // pressure_msg.header.frame_id = "pressure_sensor_link";
    // pressure_msg.header.stamp = nh.now();
    pressure_msg.data = pressure_sensor.pressure(100);
    // pressure_msg.fluid_pressure = pressure_sensor.pressure(100);

    // depth_msg.header.frame_id = "depth_sensor_link"
    // depth_msg.header.stamp = pressure_sensor.header.time;
    // depth_msg.depth = pressure_sensor.depth();

    // ps_depth_pub.publish(depth);
    ps_pressure_pub.publish(&pressure_msg);
}

// definition of callback functions
void TEastCb(const std_msgs::Int32& msg)
{
    T_EAST.speed(msg.data);
}


void TWestCb(const std_msgs::Int32& msg)
{
    T_WEST.speed(msg.data);
}

void TNorthCb(const std_msgs::Int32& msg)
{
   T_NORTH.speed(msg.data);
}

void TSouthCb(const std_msgs::Int32& msg)
{
    T_SOUTH.speed(msg.data);
}

void TNEUpCb(const std_msgs::Int32& msg)
{
    T_NORTH_EAST_UP.spin(msg.data);
}

void TSEUpCb(const std_msgs::Int32& msg)
{
    T_NORTH_WEST_UP.spin(msg.data);
}

void TNWUpCb(const std_msgs::Int32& msg)
{
    T_SOUTH_WEST_UP.spin(msg.data);
}

void TSWUpCb(const std_msgs::Int32& msg)
{
    T_SOUTH_EAST_UP.spin(msg.data);
}
