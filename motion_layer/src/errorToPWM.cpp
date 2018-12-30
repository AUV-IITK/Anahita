// Copyright 2018 AUV-IITK
#include <ros/ros.h>
#include <string>
#include <errorToPWM.h>

ErrorDescriptor::ErrorDescriptor(std::string _name): p_(0), i_(0), d_(0),
    band_(1), seed_(0), error_(0), reference_value_(0), previous_value_(0), 
    pwm_(0), previous_time_stamp_(0), current_time_stamp_(0)
{   
    this->name_ = _name;
    ROS_INFO("%s type ErrorDecriptor is constructed.", this->name_);
}

ErrorDescriptor::~ErrorDescriptor() {}

void ErrorDescriptor::setPID(float new_p, float new_i, float new_d, float new_band) {
    this->p_ = new_p;
    this->i_ = new_i;
    this->d_ = new_d;
    this->band_ = new_band;
    ROS_INFO("PID set to P: %f, I: %f, D: %f, BAND: %f", this->p_, this->i_, this->d_, this->band_);
}

void ErrorDescriptor::setReference(double _value) {
    this->reference_value_ = _value;
    // this->current_value_ = _value;

    if (this->name_ == "ANGLE") {
        if (this->reference_value_ >= 180) {
            this->reference_value_ = this->reference_value_ - 360;
        }
        else if (this->reference_value_ <= -180) {
            this->reference_value_ = this->reference_value_ + 360;
        }
    }

    ROS_INFO("%s, Reference set to: %f", this->name_, reference_value_);
}

void ErrorDescriptor::setType(std::string _name) {
    this->name_ = _name;
}

void ErrorDescriptor::errorToPWM(double _current_value) {

    this->previous_value_ = this->current_value_;
    this->current_value_ = _current_value;

    previous_time_stamp_ = current_time_stamp_;
    current_time_stamp_ = ros::Time::now().toSec();

    float derivative = 0, integral = 0;
    double dt = 0;

    // dt = current_time_stamp_ - previous_time_stamp_;
    
    if (this->name_ == "ANGLE") { dt = 0.02; }
    else { dt = 0.1; }

    if (this->name_ == "ANGLE") {
        this->error_ = this->reference_value_ - this->current_value_;
    }
    else {
        this->error_ = this->current_value_ - this->reference_value_;
    }

    if (this->name_ == "ANGLE") {
        if (this->error_ < 0) 
            this->error_value_ = this->error_ + 360;
        else
            this->error_value_ = this->error_ - 360;

        if (abs(this->error_value_) < abs(this->error_))
            this->error_ = this->error_value_;
    }

    // std::cout << this->name_ << " ERROR: " << this->error_ << std::endl;
    integral += (this->error_ * dt);
    derivative = (this->current_value_ - this->previous_value_) / dt;
    
    double output = (this->p_ * this->error_) + (this->i_ * integral) + std::abs(this->d_ * derivative);

    turningOutputPWMMapping(output);

    if (this->error_ < this->band_ && this->error_ > -this->band_) { this->pwm_ = 0; }
}

void ErrorDescriptor::turningOutputPWMMapping(float output) // to keep PWM values within a limit
{
    float maxOutput = 800, minOutput = -maxOutput;
    float scale = 400 / maxOutput;
    if (output > maxOutput)
        output = maxOutput;
    if (output < minOutput)
        output = minOutput;
    float temp = output * scale;
    int output_pwm = static_cast<int>(temp);
    if (output_pwm > 400)
        output_pwm = 400;
    if (output_pwm < -400)
        output_pwm = -400;
    this->pwm_ = output_pwm;
}

int ErrorDescriptor::getPWM() { return this->pwm_; }

double ErrorDescriptor::getCurrentValue() { return this->current_value_; }
