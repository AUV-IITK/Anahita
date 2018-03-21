#include <Arduino.h>
#include <Wire.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <auv_msgs/Pressure.h>
#include <auv_msgs/Depth.h>

#include "MS5837.h"
#include "Thruster.h"
#include "ArduinoConfig.h"

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

/*
ROBOT ORIENTATION
          FRONT
          -----
          SWAY1
          HEAVE1
            -
  SURGE1           SURGE2
            -
          HEAVE2
          SWAY2
          -----
          BACK
*/
Thruster surge1;
Thruster surge2;
Thruster heave1;
Thruster heave2;
Thruster sway1;
Thruster sway2;

// function declration to puboish pressure sensor data
void publish_pressure_data();

// callback function for thrsuters
void surge1_callback(const std_msgs::Int32& msg);
void surge2_callback(const std_msgs::Int32& msg);
void heave1_callback(const std_msgs::Int32& msg);
void heave2_callback(const std_msgs::Int32& msg);
void sway1_callback(const std_msgs::Int32& msg);
void sway2_callback(const std_msgs::Int32& msg);

// defining rosnode handle
ros::NodeHandle nh;

// declare subscribers
ros::Subscriber<std_msgs::Int32> surge1_pwm_pub("/thruster/surge1/pwm", &surge1_callback);
ros::Subscriber<std_msgs::Int32> surge2_pwm_pub("/thruster/surge2/pwm", &surge2_callback);
ros::Subscriber<std_msgs::Int32> heave1_pwm_pub("/thruster/heave1/pwm", &heave1_callback);
ros::Subscriber<std_msgs::Int32> heave2_pwm_pub("/thruster/heave2/pwm", &heave2_callback);
ros::Subscriber<std_msgs::Int32> sway1_pwm_pub("/thruster/sway1/pwm", &sway1_callback);
ros::Subscriber<std_msgs::Int32> sway2_pwm_pub("/thruster/sway1/pwm", &sway2_callback);

// declare publishers
auv_msgs::Pressure depth;
auv_msgs::Pressure pressure;
ros::Publisher ps_depth_pub("/pressure_sensor/depth", &depth);
ros::Publisher ps_pressure_pub("/pressure_sensor/pressure", &pressure);

void setup()
{
    //setting up thruster pins
    surge1.setup(SURGE1_PWM, SURGE1_IN_A, SURGE1_IN_B);
    surge2.setup(SURGE2_PWM, SURGE2_IN_A, SURGE2_IN_B);
    heave1.setup(HEAVE1_PWM, HEAVE1_IN_A, HEAVE1_IN_B);
    heave2.setup(HEAVE2_PWM, HEAVE2_IN_A, HEAVE2_IN_B);
    sway1.setup(SWAY1_PWM, SWAY1_IN_A, SWAY1_IN_B);
    sway2.setup(SWAY2_PWM, SWAY2_IN_A, SWAY2_IN_B);

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
    nh.subscribe(surge1_pwm_pub);
    nh.subscribe(surge2_pwm_pub);
    nh.subscribe(heave1_pwm_pub);
    nh.subscribe(heave2_pwm_pub);
    nh.subscribe(sway1_pwm_pub);
    nh.subscribe(sway2_pwm_pub);
    // publisher
    nh.advertise(ps_depth_pub);
    nh.advertise(ps_pressure_pub);

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
    depth.header.frame_id = "depth_sensor_link"
    depth.header.stamp = nh.now();
    depth.depth = pressure_sensor.depth();

    pressure_sensor.header.frame_id = "depth_sensor_link";
    pressure_sensor.header.time = depth.header.time;
    pressure_sensor.pressure = pressure_sensor.pressure(100);

    ps_depth_pub.publish(depth);
    ps_pressure_pub.publish(pessure);
}

// function definitions for callback
void surge1_callback(const std_msgs::Int32& msg)
{
   surge1.spin(msg.data);
}

void surge2_callback(const std_msgs::Int32& msg)
{
    surge2.spin(msg.data);
}

void heave1_callback(const std_msgs::Int32& msg)
{
   heave1.spin(msg.data);
}

void heave2_callback(const std_msgs::Int32& msg)
{
    heave2.spin(msg.data);
}

void sway1_callback(const std_msgs::Int32& msg)
{
   sway1.spin(msg.data);
}

void sway2_callback(const std_msgs::Int32& msg)
{
    sway2.spin(msg.data);
}
