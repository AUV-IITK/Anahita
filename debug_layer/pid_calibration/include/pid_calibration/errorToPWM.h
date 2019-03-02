// Copyright 2018 AUV-IITK
#ifndef ERRORDESCRIPTOR_H
#define ERRORDESCRIPTOR_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <string>

class ErrorDescriptor {
    private: std::string name_;
    private: int pwm_;
    
    private: double p_;
    private: double i_;
    private: double d_;
    private: double band_;

    private: double current_value_;
    private: double previous_value_;
    private: double error_;
    private: double error_value_;
    private: double reference_value_;
    
    private: double seed_;
    private: double previous_time_stamp_ = 0;
    private: double current_time_stamp_ = 0;
    
    public: ErrorDescriptor(std::string _name);
    public: ~ErrorDescriptor();

    public: void setPID(float new_p, float new_i, float new_d, float new_band);
    public: void setType(std::string _name);
    public: void setReference(double _value);

    public: void errorToPWM(double _current_value);
    private: void turningOutputPWMMapping(float output);
    public: int getPWM();
    public: double getCurrentValue();
};
#endif // !ERRORDESCRIPTOR_H
