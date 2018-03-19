#include "Arduino.h"
#include "../include/ArduinoThrust.h"

using namespace std;

void ArduinoThrust::setPins(int A1, int D1, int D2)
{
    An1_ = A1;
    D1_ = D1;
    D2_ = D2;
    pinMode(An1_, OUTPUT);
    pinMode(D1_, OUTPUT);
    pinMode(D2_, OUTPUT);
}

void ArduinoThrust::ON(int pwm)
{
    if(pwm > 0)
    {
        digitalWrite(D1_, HIGH);
        digitalWrite(D2_, LOW);
    }
    else if(pwm < 0)
    {
        digitalWrite(D1_, LOW);
        digitalWrite(D2_, HIGH);
    }
    else
    {
        digitalWrite(D1_, LOW);
        digitalWrite(D2_, LOW);
    }
    // inverse mapping in the motor driver
    analogWrite(An1_, 255 - abs(pwm));
}
