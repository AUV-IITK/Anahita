#ifndef Thruster_H
#define Thruster_H
using namespace std;

enum class Mode { STOP, LOW, MED, HIGH };

// structure to store the three speed mode values in one direction
struct ThrusterMode
{
  int High;
  int Med;
  int Low;
};

class Thruster
{
	public:
    // tells the speed mode of the thruster
    int mode(int P);

    // returns the value of pwm_
    int getPWM();

    //it sets the struct values of any instance of this class
    void setValues(int HC, int MC,int LC, int HAC, int MAC, int LAC);

    // classifies the obtained pwm variable to given specified speed modes
    void calibration(int PWM, int M);

  private:
    int pwm_;                // stores the value of pwm corresponding to each thruster
    ThrusterMode clock_;     // stores the value of high,med and low pwm values for clockwise direction
    ThrusterMode anticlock_; // stores the value of high,med and low pwm values for anticlockwise direction
};

#endif
