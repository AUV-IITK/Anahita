#include "Arduino.h"
#include "Thruster.h"

// constructor call without current sensor setup
Thruster::Thruster(int pwm_pin, int thruster_pinA, int thruster_pinB)
{
    pwm_pin_ = pwm_pin;
    thruster_pinA_ = thruster_pinA;
    thruster_pinB_ = thruster_pinB;
    current_sensor_pin_ = -1;
}

// constructor call with current sensor setup
Thruster::Thruster(int pwm_pin, int thruster_pinA, int thruster_pinB, int current_sensor_pin)
{
    pwm_pin_ = pwm_pin;
    thruster_pinA_ = thruster_pinA;
    thruster_pinB_ = thruster_pinB;
    current_sensor_pin_ = current_sensor_pin;
}

//sets the direction and pwm pins for the thrusters
void Thruster::setup()
{
  pinMode(pwm_pin_, OUTPUT);
  pinMode(thruster_pinA_, OUTPUT);
  pinMode(thruster_pinB_, OUTPUT);
}

// read the current value from the thruster driver
void Thruster::readCurrent()
{
    if(current_sensor_pin_ != -1)
        val_ = analogRead(current_sensor_pin_);
}

// set the thruster to motion depending on pwm input
void Thruster::spin(int pwm)
{
    if(pwm > 0)
    {
        digitalWrite(thruster_pinA_, HIGH);
        digitalWrite(thruster_pinB_, LOW);
    }
    else if(pwm < 0)
    {
        digitalWrite(thruster_pinA_, LOW);
        digitalWrite(thruster_pinB_, HIGH);
    }
    else
    {
      digitalWrite(thruster_pinA_, LOW);
      digitalWrite(thruster_pinB_, LOW);
    }
    // inverse mapping in the thruster driver
    analogWrite(pwm_pin_, 255 - abs(pwm));
}
