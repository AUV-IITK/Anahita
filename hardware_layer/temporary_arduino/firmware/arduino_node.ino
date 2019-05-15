#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MS5837.h>

#define servoEastPin 40 // pin definitions for forward thrusters
#define servoWestPin 43

#define servoNorthSwayPin 31 // pin definitions for sideward thrusters
#define servoSouthSwayPin 32

#define servoNorthWestUpPin 28 // pin definitions for upward thrusters
#define servoSouthWestUpPin 33
#define servoNorthEastUpPin 27 
#define servoSouthEastUpPin 30

#define permanentGround1 29
#define permanentGround2 26
#define permanentGround3 41
#define permanentGround4 42 

MS5837 pressure_sensor;

Servo servoEast;
Servo servoWest;
Servo servoNorthSway;
Servo servoSouthSway;
Servo servoNorthWestUp;
Servo servoNorthEastUp;
Servo servoSouthWestUp;
Servo servoSouthEastUp;

int forward_right = 0;
int forward_left = 0;
int sideward_front = 0;
int sideward_back = 0;
int upward_north_east = 0;
int upward_north_west = 0;
int upward_south_east = 0;
int upward_south_west = 0;

String readString;
int time = 0;
bool stringComplete = false;

float depth_reading = 0;

int ESC_Zero = 1500;

void setup()
{

    Serial.begin(115200);  // initialize serial communications at 115200 bps
    while (!Serial) {}
    readString.reserve(200);

    servoEast.attach(servoEastPin);
    servoEast.writeMicroseconds(ESC_Zero);
    servoNorthSway.attach(servoNorthSwayPin);
    servoNorthSway.writeMicroseconds(ESC_Zero);
    servoWest.attach(servoWestPin);
    servoWest.writeMicroseconds(ESC_Zero);
    servoSouthSway.attach(servoSouthSwayPin);
    servoSouthSway.writeMicroseconds(ESC_Zero);

    servoNorthWestUp.attach(servoNorthWestUpPin);
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
    servoNorthEastUp.attach(servoNorthEastUpPin);
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
    servoSouthWestUp.attach(servoSouthWestUpPin);
    servoSouthWestUp.writeMicroseconds(ESC_Zero);
    servoSouthEastUp.attach(servoSouthEastUpPin);
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
    
    pinMode(permanentGround1, OUTPUT);
    pinMode(permanentGround2, OUTPUT);
    pinMode(permanentGround3, OUTPUT);
    pinMode(permanentGround4, OUTPUT);

    digitalWrite(permanentGround1, LOW);
    digitalWrite(permanentGround2, LOW);
    digitalWrite(permanentGround3, LOW);
    digitalWrite(permanentGround4, LOW);
        
    delay(7000);

    Wire.begin();
    pressure_sensor.init();
    pressure_sensor.setFluidDensity(997);
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    decode(readString);
    Serial.print(depth_reading);
    // clear the string:
    readString = "";
    stringComplete = false;
  }
  delay(10);
  pwmPublish();
  getSensorData();
}

void pwmPublish() {
  servoEast.writeMicroseconds(forward_right);
  servoWest.writeMicroseconds(forward_left);
  servoNorthSway.writeMicroseconds(sideward_front);
  servoSouthSway.writeMicroseconds(sideward_back);
  servoNorthWestUp.writeMicroseconds(upward_north_west);
  servoNorthEastUp.writeMicroseconds(upward_north_east);
  servoSouthWestUp.writeMicroseconds(upward_south_west);
  servoSouthEastUp.writeMicroseconds(upward_south_east); 
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char) Serial.read();
    // add it to the inputString:
    readString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void decode (String pwm_str) {

   int i = 0;
   int j = 0;
   String pwm = "";
   int a[8] = {0};

   while (pwm_str[i] != '\n') {
      if (pwm_str[i] == ',') {
        a[j] = pwm.toInt();
        j = j + 1;
        i = i + 1;
        pwm = "";
        continue;
      }
      pwm = pwm + pwm_str[i];
      i = i + 1;
   }

  forward_right = ESC_Zero + a[3];
  forward_left = ESC_Zero + a[2];
  sideward_front = ESC_Zero + a[0];
  sideward_back = ESC_Zero + a[1];
  upward_north_east = ESC_Zero + a[4];
  upward_north_west = ESC_Zero + a[5];
  upward_south_east = ESC_Zero + a[6];
  upward_south_west = ESC_Zero + a[7];
}

void getSensorData() {
  pressure_sensor.read();
  depth_reading = -100*pressure_sensor.depth();
}
