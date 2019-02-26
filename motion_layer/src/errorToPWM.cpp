// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <string>
#include <errorToPWM.h>

ErrorDescriptor::ErrorDescriptor(std::string _name): p_(0), i_(0), d_(0),
    band_(1), seed_(0), error_(0), reference_value_(0), previous_value_(0), 
    pwm_(0), previous_time_stamp_(0), current_time_stamp_(0)
{   
    name_ = _name;
    std::cout << name_ << " type ErrorDecriptor is constructed" << std::endl;
}

ErrorDescriptor::~ErrorDescriptor() {}

void ErrorDescriptor::setPID(float new_p, float new_i, float new_d, float new_band) {
    p_ = new_p;
    i_ = new_i;
    d_ = new_d;
    band_ = new_band;
    ROS_INFO("PID set to P: %f, I: %f, D: %f, BAND: %f", p_, i_, d_, band_);
}

void ErrorDescriptor::setReference(double _value) {
    reference_value_ = _value;
    // current_value_ = _value;

    if (name_ == "YAW" || name_ == "PITCH" || name_ == "ROLL") {
        if (reference_value_ >= 180) {
            reference_value_ = reference_value_ - 360;
        }
        else if (reference_value_ <= -180) {
            reference_value_ = reference_value_ + 360;
        }
    }

    std::cout << name_ << ", Reference set to: " << reference_value_ << std::endl;
}

void ErrorDescriptor::setType(std::string _name) {
    name_ = _name;
}

void ErrorDescriptor::errorToPWM(double _current_value) {

    previous_value_ = current_value_;
    current_value_ = _current_value;

    previous_time_stamp_ = current_time_stamp_;
    current_time_stamp_ = ros::Time::now().toSec();

    float derivative = 0, integral = 0;
    double dt = 0;

    // dt = current_time_stamp_ - previous_time_stamp_;
    
    if (name_ == "YAW") { dt = 0.02; }
    else { dt = 0.05; }

    error_ = current_value_ - reference_value_;

    if (name_ == "YAW") {
        if (error_ < 0) 
            error_value_ = error_ + 360;
        else
            error_value_ = error_ - 360;

        if (abs(error_value_) < abs(error_))
            error_ = error_value_;
    }

    // std::cout << name_ << " ERROR: " << error_ << std::endl;
    integral += (error_ * dt);
    derivative = (current_value_ - previous_value_) / dt;
    
    double output = (p_ * error_) + (i_ * integral) + std::abs(d_ * derivative);

    turningOutputPWMMapping(output);

    if (error_ < band_ && error_ > -band_) { pwm_ = 0; }
}

void ErrorDescriptor::turningOutputPWMMapping(float output) // to keep PWM values within a limit
{
    float maxOutput = 800, minOutput = -maxOutput;
    float scale = 200 / maxOutput;
    if (output > maxOutput)
        output = maxOutput;
    if (output < minOutput)
        output = minOutput;
    float temp = output * scale;
    int output_pwm = static_cast<int>(temp);
    if (output_pwm > 200)
        output_pwm = 200;
    if (output_pwm < -200)
        output_pwm = -200;
    pwm_ = output_pwm;
}

int ErrorDescriptor::getPWM() { return pwm_; }

double ErrorDescriptor::getCurrentValue() { return current_value_; }
