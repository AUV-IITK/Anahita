#ifndef _Thruster_H_INCLUDED_
#define _Thruster_H_INCLUDED_

using namespace std;

class Thruster
{
  public:
    // constructors
    Thruster(int pwm_pin, int thruster_pinA, int thruster_pinB);
    Thruster(int pwm_pin, int thruster_pinA, int thruster_pinB, int current_sensor_pin);

    //sets the direction and pwm pins for the thrusters
    void setup();

    // read the current value from the thruster driver
    void readCurrent();

    // set the thruster to motion depending on pwm input
    void spin(int pwm);

  private:
    int val_;                    // current sensor output
    int pwm_pin_;                // pwm pin number
    int current_sensor_pin_;     // current sensor pin number
    int thruster_pinA_;          // direction pin 1
    int thruster_pinB_;          // direction pin 2
};

#endif
