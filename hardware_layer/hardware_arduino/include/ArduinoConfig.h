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

#define servoPinEast 40      // pin definitions for forward thrusters
#define servoPinWest 43

#define servoPinNorthSway 31   // pin definitions for sideward thrusters
#define servoPinSouthSway 30

#define servoPinNorthWestUp 6   // pin definitions for upward thrusters
#define servoPinSouthWestUp 7
#define servoPinNorthEastUp 8
#define servoPinSouthEastUp 9

#define permanentGround_1 26   // he pins are defined to keep them permamenent ground for ESCs
#define permanentGround_2 29
#define permanentGround_3 41
#define permanentGround_4 42

// #define pwmPinNorthWestUp 6     // pin definitions for upward east side thrusters
// #define pwmPinSouthWestUp 7
// #define directionPinNorthWestUp1 8
// #define directionPinNorthWestUp2 9
// #define directionPinSouthWestUp1 10
// #define directionPinSouthWestUp2 11

// #define pwmPinNorthEastUp 12      // pin definitions for upward west side thrusters
// #define pwmPinSouthEastUp 13
// #define directionPinNorthEastUp1 14
// #define directionPinNorthEastUp2 15
// #define directionPinSouthEastUp1 16
// #define directionPinSouthEastUp2 17

#define analogPinPressureSensor A0      //pin definition for depth sensor

#endif
