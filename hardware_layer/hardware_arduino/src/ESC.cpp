#include "../include/ESC.h"

ESC::ESC(byte ESC_pin, int maxReverse, int maxForward, int armVal)
{
    ESCPin_ = ESC_pin;
    maxRev_ = maxReverse;
    maxFor_ = maxForward;
    valArm_ = armVal;
}

// Calibration is done by sending the maximum and minimum PWM signal to the ESC i.e. maxRev_ and maxFor_ value
void ESC::calibrate()
{
    ESC_.attach(ESC_pin);              // attaches the ESC on pin oPin to the ESC object
    ESC_.writeMicroseconds(maxFor_);
    delay(calib_delay_);
    ESC_.writeMicroseconds(minRev_);
    delay(calib_delay_);
    arm();
}

// By arming an ESC, we pass the arm value that is the stop signal to the ESC
void ESC::arm()
{
    ESC_.attach(ESC_pin);              // attaches the ESC on pin oPin to the ESC object
    ESC_.writeMicroseconds (valArm_);
    delay(calib_delay);
}

//speed method contorls the speed of the T200 thruster (considering that outputESC is in the range of the ESC
void ESC::speed(int outputESC)
{
    ESC_.writeMicroseconds(outputESC);
}
