#ifndef __ArduinoConfig_H_INCLUDED__
#define __ArduinoConfig_H_INCLUDED__

/*
ROBOT ORIENTATION
                       NORTH
                    -----------
       NORTH-WEST-UP|         |NORTH-EAST-UP
                    |         |
                    |    -    |
               WEST |    -    | EAST
                    |    -    |
                    |         |
       SOUTH-WEST-UP|         |SOUTH-EAST-UP
                    -----------  
                       SOUTH
*/

#define servoEastPin 40       // pin definitions for forward thrusters
#define servoWestPin 43

#define servoNorthSwayPin 31   // pin definitions for sideward thrusters
#define servoSouthSwayPin 30

#define servoNorthWestUpPin 32  // pin definitions for upward thrusters
#define servoSouthWestUpPin 33
#define servoNorthEastUpPin 29   
#define servoSouthEastUpPin 26

#define permanentGround1 28
#define permanentGround2 27
#define permanentGround3 41
#define permanentGround4 43

#define analogPinPressureSensor A0      //pin definition for depth sensor

#endif
