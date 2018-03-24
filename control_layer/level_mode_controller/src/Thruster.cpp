#include "level_mode_controller/Thruster.h"

using namespace std;

// tells the speed mode of the thruster
int Thruster::mode(int P)
{
    if(P == 0)
        return (int) Mode::STOP;
    else if(P < 180)
        return (int) Mode::LOW;
    else if(P >= 180 && P < 220)
        return (int) Mode::MED;
    else
        return (int) Mode::HIGH;
}

// returns the value of pwm_
int Thruster::getPWM(){
  return pwm_;
}

// it sets the struct values of any instance of this class
void Thruster::setValues(int HC, int MC, int LC, int HAC, int MAC, int LAC)
{
    clock_.High = HC;
    clock_.Med = MC;
    clock_.Low = LC;
    anticlock_.High = HAC;
    anticlock_.Med = MAC;
    anticlock_.Low = LAC;
}

// classifies the obtained pwm variable to given specified speed modes
void Thruster::calibration(int PWM, int M)
{
    bool positive = true;
    if(M == 0 && PWM == 0)
    {
        pwm_ = 0;
        return;
    }
    if(PWM < 0)
        positive = false;
    if(positive == true)
    {
        switch(M)
        {
            case 1: pwm_ = clock_.Low;
                break;
            case 2: pwm_ = clock_.Med;
                break;
            case 3: pwm_ = clock_.High;
                break;
        }
    }
    else
    {
        switch(M)
        {
            case 1: pwm_ = anticlock_.Low;
                break;
            case 2: pwm_ = anticlock_.Med;
                break;
            case 3: pwm_ = anticlock_.High;
                break;
        }
    }
    return;
}
