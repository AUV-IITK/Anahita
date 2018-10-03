#include <Thruster.h>
#include "Arduino.h"

// constructor call without current sensor setup
Thruster::Thruster(int pwm_pin, int direction_pinA, int direction_pinB)
{
    pwm_pin_ = pwm_pin;
    direction_pinA_ = direction_pinA;
    direction_pinB_ = direction_pinB;
    current_sensor_pin_ = -1;
}

// constructor call with current sensor setup
Thruster::Thruster(int pwm_pin, int direction_pinA, int direction_pinB, int current_sensor_pin)
{
    pwm_pin_ = pwm_pin;
    direction_pinA_ = direction_pinA;
    direction_pinB_ = direction_pinB;
    current_sensor_pin_ = current_sensor_pin;
}

//sets the direction and pwm pins for the thrusters
void Thruster::setup()
{
  pinMode(pwm_pin_, OUTPUT);
  pinMode(direction_pinA_, OUTPUT);
  pinMode(direction_pinB_, OUTPUT);
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
        digitalWrite(direction_pinA_, HIGH);
        digitalWrite(direction_pinB_, LOW);
    }
    else if(pwm < 0)
    {
        digitalWrite(direction_pinA_, LOW);
        digitalWrite(direction_pinB_, HIGH);
    }
    else
    {
      digitalWrite(direction_pinA_, LOW);
      digitalWrite(direction_pinB_, LOW);
    }
    // inverse mapping in the thruster driver
    analogWrite(pwm_pin_, 255 - abs(pwm));
}
