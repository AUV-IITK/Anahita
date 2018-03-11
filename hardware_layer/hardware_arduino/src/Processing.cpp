#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <math.h>
#include "thruster.h"

//instances of the class thruster defined in header file
thruster TEAST,TWEST,TNORTHUP,TSOUTHUP,TNORTHSWAY,TSOUTHSWAY;

bool isMovingForward = false; //variable to distinguish for turning with forward or sideward thrusters

ros::NodeHandle n;

//Topics for publishing data and communicating to the arduino
ros::Publisher pubPwmEast = n.advertise<std_msgs::Int32>("/ard/east", 1000);
ros::Publisher pubPwmWest = n.advertise<std_msgs::Int32>("/ard/west", 1000);
ros::Publisher pubPwmNorthUp = n.advertise<std_msgs::Int32>("/ard/northup", 1000);
ros::Publisher pubPwmSouthUp = n.advertise<std_msgs::Int32>("/ard/southup", 1000);
ros::Publisher pubPwmNorthSway = n.advertise<std_msgs::Int32>("/ard/northsway", 1000);
ros::Publisher pubPwmSouthSway = n.advertise<std_msgs::Int32>("/ard/southsway", 1000);

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
    if(!isMovingForward)    //checking if forward thrusters are not preoccupied
    {
        TEAST.calibration(pwm, TEAST.mode(abs(pwm)));
        TWEST.calibration(-pwm, TWEST.mode(abs(pwm)));
        v.data = TEAST.getPWM();
        pubPwmEast.publish(v);
        v.data = TWEST.getPWM();
        pubPwmWest.publish(v);
    }
    else    ///forward thrusters are already in motion
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
    ros::init(argc, argv, "Processing");

    //Topics receiving pwm values from the motion library
    ros::Subscriber subPwmForward = n.subscribe("/pwm/forward", 1000, ForwardCB);
    ros::Subscriber subPwmSideward = n.subscribe("/pwm/sideward", 1000, SidewardCB);
    ros::Subscriber subPwmUpward= n.subscribe("/pwm/upward", 1000, UpwardCB);
    ros::Subscriber subPwmTurn = n.subscribe("/pwm/turn", 1000, TurnCB);


    ros::spin();
    return 0;
}
