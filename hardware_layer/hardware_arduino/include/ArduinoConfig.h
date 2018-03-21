#ifndef __ArduinoConfig_H_INCLUDED__
#define __ArduinoConfig_H_INCLUDED__

/*
ROBOT ORIENTATION
          FRONT
          -----
          SWAY1
          HEAVE1
            -
  SURGE1           SURGE2
            -
          HEAVE2
          SWAY2
          -----
          BACK
*/

// for sway motion
#define SWAY1_PWM 5
#define SWAY1_IN_A 27
#define SWAY1_IN_B 26

#define SWAY2_PWM 4
#define SWAY2_IN_A 29
#define SWAY2_IN_B 28

// for surge motion
#define SURGE1_PWM 3
#define SURGE1_IN_A 32
#define SURGE1_IN_B 33

#define SURGE2_PWM 2
#define SURGE2_IN_A 30
#define SURGE2_IN_B 31

// for heave motion
#define HEAVE1_PWM 6
#define HEAVE1_IN_A 24
#define HEAVE1_IN_B 25

#define HEAVE2_PWM 7
#define HEAVE2_IN_A 22
#define HEAVE2_IN_B 23

// for pressure sensor
#define PRESSURE_SENSOR_PIN A0

#endif
