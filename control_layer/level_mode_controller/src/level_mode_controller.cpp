#include <ros/ros.h>
#include <std_msgs/Int32.h>

#include "level_mode_controller/Thruster.h"
#include <math.h>

//instances of the class Thruster defined in header file
Thruster TEAST,TWEST,TNORTHUP,TSOUTHUP,TNORTHSWAY,TSOUTHSWAY;

bool isMovingForward = false; //variable to distinguish for turning with forward or sideward Thrusters

ros::NodeHandle nh;

//Topics for publishing data and communicating to the arduino
ros::Publisher pubPwmEast = nh.advertise<std_msgs::Int32>("/ard/east", 1000);
ros::Publisher pubPwmWest = nh.advertise<std_msgs::Int32>("/ard/west", 1000);
ros::Publisher pubPwmNorthUp = nh.advertise<std_msgs::Int32>("/ard/northup", 1000);
ros::Publisher pubPwmSouthUp = nh.advertise<std_msgs::Int32>("/ard/southup", 1000);
ros::Publisher pubPwmNorthSway = nh.advertise<std_msgs::Int32>("/ard/northsway", 1000);
ros::Publisher pubPwmSouthSway = nh.advertise<std_msgs::Int32>("/ard/southsway", 1000);

void ForwardCB(const std_msgs::Int32& msg)  //callback function
{
    int pwm = msg.data;
    TEAST.calibration(pwm, TEAST.mode(abs(pwm)));    //classifying and storing the value in private pwm variable
    TWEST.calibration(pwm, TWEST.mode(abs(pwm)));
    std_msgs::Int32 v;
    v.data = TEAST.getPWM();
    pubPwmEast.publish(v);  //publishing on concerned topics linked to arduino
    v.data = TWEST.getPWM();
    pubPwmWest.publish(v);
    isMovingForward = true;
}

void SidewardCB(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    TNORTHSWAY.calibration(pwm, TNORTHSWAY.mode(abs(pwm)));
    TSOUTHSWAY.calibration(pwm, TSOUTHSWAY.mode(abs(pwm)));
    std_msgs::Int32 v;
    v.data = TNORTHSWAY.getPWM();
    pubPwmNorthSway.publish(v);
    v.data = TSOUTHSWAY.getPWM();
    pubPwmSouthSway.publish(v);
    isMovingForward = false;
}

void UpwardCB(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    TNORTHUP.calibration(pwm, TNORTHUP.mode(abs(pwm)));
    TSOUTHUP.calibration(pwm, TSOUTHUP.mode(abs(pwm)));
    std_msgs::Int32 v;
    v.data = TNORTHUP.getPWM();
    pubPwmNorthUp.publish(v);
    v.data = TSOUTHUP.getPWM();
    pubPwmSouthUp.publish(v);
    isMovingForward = false;
}

void TurnCB(const std_msgs::Int32& msg)
{
    int pwm = msg.data;
    std_msgs::Int32 v;
    if(!isMovingForward)    //checking if forward Thrusters are not preoccupied
    {
        TEAST.calibration(pwm, TEAST.mode(abs(pwm)));
        TWEST.calibration(-pwm, TWEST.mode(abs(pwm)));
        v.data = TEAST.getPWM();
        pubPwmEast.publish(v);
        v.data = TWEST.getPWM();
        pubPwmWest.publish(v);
    }
    else    ///forward Thrusters are already in motion
    {
        TNORTHSWAY.calibration(pwm, TNORTHSWAY.mode(abs(pwm)));
        TSOUTHSWAY.calibration(-pwm, TSOUTHSWAY.mode(abs(pwm)));
        v.data = TNORTHSWAY.getPWM();
        pubPwmNorthSway.publish(v);
        v.data = TSOUTHSWAY.getPWM();
        pubPwmSouthSway.publish(v);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_mode_controller");

    //Topics receiving pwm values from the motion library
    ros::Subscriber subPwmForward = nh.subscribe("/pwm/forward", 1000, ForwardCB);
    ros::Subscriber subPwmSideward = nh.subscribe("/pwm/sideward", 1000, SidewardCB);
    ros::Subscriber subPwmUpward= nh.subscribe("/pwm/upward", 1000, UpwardCB);
    ros::Subscriber subPwmTurn = nh.subscribe("/pwm/turn", 1000, TurnCB);

    ros::spin();
    return 0;
}
