#ifndef ArduinoThrust_H
#define ArduinoThrust_H

using namespace std;

class ArduinoThrust
{
    public:
    
        //sets the direction and pwm pins for the thrusters and initiate them as outputs
        void setPins(int A1, int D1, int D2);
    
        //applies analogwrite and digitalwrite functions in arduino
        void ON(int pwm);
    
    private:
        int An1_;   //stores the analog pin number for the thruster on arduino
        int D1_;    //stores the direction pin number 1 for the thruster on arduino
        int D2_;    //stores the direction pin number 2 for the thruster on arduino
    
};

#endif
