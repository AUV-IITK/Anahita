#include "thruster.h"

#define HIGH 3
#define MED 2
#define LOW 1

using namespace std;

int thruster::mode(int P)
{
    if(P == 0)
        return 0;
    else if(P < 180)
        return LOW;
    else if(P >= 180 && P < 220)
        return MED;
    else
        return HIGH;
}

int thruster::getPWM(){
  return pwm_;
}

void thruster::setValues(int HC, int MC, int LC, int HAC, int MAC, int LAC)
{
    clock_.High = HC;
    clock_.Med = MC;
    clock_.Low = LC;
    anticlock_.High = HAC;
    anticlock_.Med = MAC;
    anticlock_.Low = LAC;
}

void thruster::calibration(int PWM, int M)
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
