#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <MS5837.h>

#define MDPin1 25
#define MDPin2 24

#define servoEastPin 40       // pin definitions for forward thrusters
#define servoWestPin 43

#define servoNorthSwayPin 31   // pin definitions for sideward thrusters
#define servoSouthSwayPin 32
//31 30 32 33
#define servoNorthWestUpPin 28// pin definitions for upward thrusters
#define servoSouthWestUpPin 33
#define servoNorthEastUpPin 27 
#define servoSouthEastUpPin 30

#define permanentGround1 29
#define permanentGround2 26
#define permanentGround3 41
#define permanentGround4 42

#define TorpedoPin1 38
#define TorpedoPin2 39

#define MDServoPin 3      //pin definition for depth sensor

Servo servoEast;
Servo servoWest;
Servo servoNorthSway;
Servo servoSouthSway;
Servo servoNorthWestUp;
Servo servoNorthEastUp;
Servo servoSouthWestUp;
Servo servoSouthEastUp;
Servo servoMarkerDropper;

void TEast(const int data);
void TWest(const int data);
void TNorth(const int data);
void TSouth(const int data);
void TNEUp(const int data);
void TNWUp(const int data);
void TSEUp(const int data);
void TSWUp(const int data);
void MDCb(const int msg);
void SOVCb(const int msg);
void read_pressure_data();

int ESC_Zero = 1500;
int deadZone = 25;
int torpedo_count = 0;
double depth_reading;

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

// function declration to puboish pressure sensor data
void publish_pressure_data();

int forward_right = 0;
int forward_left = 0;
int sideward_front = 0;
int sideward_back = 0;
int upward_north_east = 0;
int upward_north_west = 0;
int upward_south_east = 0;
int upward_south_west = 0;
int marker_dropper = 0;
int torpedo = 0;

// Going to use a message string
String message="";
bool stringComplete = false;
const int COMMAND_LENGTH = 18;

uint32_t delayMilliseconds = 50;
uint32_t prev_time = millis();

void split(String messages, String *splitMessages, char delimiter=',');
void actuatorLoop();
void sensorLoop();


void setup()
{  
    Serial.begin(115200);
    while(!Serial)
    {
      // Waiting for a connection to be generated, needed for delays caused by USB
    }
    message.reserve(200);
    //reserving initial length, strings are a headache in C

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
    
    pinMode(TorpedoPin1, OUTPUT); 
    pinMode(TorpedoPin2 ,OUTPUT);
   
    digitalWrite(TorpedoPin1,LOW);
    digitalWrite(TorpedoPin2,LOW); 
    
    servoMarkerDropper.attach(MDServoPin);
    
    delay(7000);
   
    // setting up pressure sensor
    Wire.begin();
    // We can't continue with the rest of the program unless we can initialize the sensor
    pressure_sensor.init();
    pressure_sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)*/   
}

void loop()
{
    static unsigned long prev_pressure_time = 0;

    // this block publishes the pressure sensor data based on defined rate
    if ((millis() - prev_pressure_time) >= (1000 / PRESSURE_PUBLISH_RATE))
    {
        // publish_pressure_data();
        prev_pressure_time = millis();
    }
    
    delay(10);
    
    //Just calling the two loops which will handle i/o as well
    actuatorLoop();
    sensorLoop();
}

//this function takes any input serial to a string buffer
//runs without any delay() calls, and thus tries to achieve maximum loop_rate as possible
void serialEvent(){
  
  if(stringComplete){
    message="";
    stringComplete = false;
    //this is for complete reading of a string, we clear message and counter
  }

  while (Serial.available()){
    //get the new byte from serial
    char inputChar = (char) Serial.read();
    message += inputChar;
    //if the incoming character is a newline, set a flag
    if (inputChar == '\n')
    {
      stringComplete = true;
      return;
    }
    //no return statement outside since we canot miss anything
  }
}

void actuatorLoop(){
  // If we have recieved a full message, actuate based upon it
  if(stringComplete){
    String splitMessages[COMMAND_LENGTH];
    for(int i = 0; i < COMMAND_LENGTH; i++){
      splitMessages[0] = "";
    }
    split(message, splitMessages);

    // We've already used this message
    message = "";
    stringComplete = false;
    // status, blue, white, red
    if(splitMessages[0] != "0"){
      return;
    }
    
    TEast(splitMessages[1].toInt());
    TWest(splitMessages[2].toInt());
    TNorth(splitMessages[3].toInt());
    TSouth(splitMessages[4].toInt());
    TNEUp(splitMessages[5].toInt());
    TNWUp(splitMessages[6].toInt());
    TSEUp(splitMessages[7].toInt());
    TSWUp(splitMessages[8].toInt());
    MDCb(splitMessages[9].toInt());
    SOVCb(splitMessages[10].toInt());


    if(torpedo_count == 0)
    {
      digitalWrite(TorpedoPin1, LOW);
      digitalWrite(TorpedoPin2, LOW);
    }
    digitalWrite(TorpedoPin1, HIGH);
  }

}
  

void sensorLoop(){

  //Using the idea of printing data in CSV format
  read_pressure_data();
  Serial.print(0);
  Serial.print(',');
  Serial.print(depth_reading);
  Serial.print('\n');

  // Wait until done writing.
    Serial.flush();
}


// function definition for publish_pressure_data
void read_pressure_data()
{
    pressure_sensor.read();
   // pressure_msg.data = pressure_sensor.pressure();

    /** Depth returned in meters (valid for operation in incompressible
    *  liquids only. Uses density that is set for fresh or seawater.
    */
    depth_reading = -100*pressure_sensor.depth(); //convert to centimeters

}

void MDCb(const int msg)
{
  int data = msg;
  int pos = 0;
  if(data == 1)
  {
    for(pos = 0; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
  }
  if(data == 2)
  {
    for(pos = 170; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
  }
 if(data == -1)
 { 
   for(pos = 0; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
 }
 if(data == -2)
 {
   for(pos = 0; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
 }
}

void SOVCb(const int msg)
{
  int data = msg;
  int pos = 0;
  if(data == 1)
  {
    torpedo_count = 1;
    digitalWrite(TorpedoPin1, HIGH);
    delay(100);
    digitalWrite(TorpedoPin1, LOW);
    torpedo_count = 0;
  }
  else if(data == 2)
  {
    torpedo_count = 1;
    digitalWrite(TorpedoPin2, HIGH);
    delay(100);
    digitalWrite(TorpedoPin2, LOW);
    torpedo_count = 0;
  }
}

void TEast(const int data)
{
  if(data>0)
    servoEast.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoEast.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoEast.writeMicroseconds(ESC_Zero);
}
void TWest(const int data)
{
  if(data>0)
    servoWest.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoWest.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoWest.writeMicroseconds(ESC_Zero);
}
void TNorth(const int data)
{
  if(data>0)
    servoNorthSway.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthSway.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthSway.writeMicroseconds(ESC_Zero);
}
void TSouth(const int data)
{
  if(data>0)
    servoSouthSway.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthSway.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthSway.writeMicroseconds(ESC_Zero);
}
void TNEUp(const int data)
{
  if(data>0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
}
void TNWUp(const int data)
{
  if(data>0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
}
void TSEUp(const int data)
{
  if(data>0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
}
void TSWUp(const int data)
{
  if(data>0)
    servoSouthWestUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthWestUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthWestUp.writeMicroseconds(ESC_Zero);
}

void split(String messages, String* splitMessages,  char delimiter){
  int indexOfComma = 0;
  for(int i = 0; messages.indexOf(delimiter, indexOfComma) > 0; i++){
    int nextIndex = messages.indexOf(delimiter, indexOfComma+1);
    String nextMessage;

    // The first message doesn't have an initial comma, so account for that.
    if(indexOfComma == 0){
      indexOfComma = -1;
    }
    if(nextIndex == -1){
      nextMessage = messages.substring(indexOfComma+1);
    }else{
      nextMessage = messages.substring(indexOfComma+1, nextIndex);
    }
    splitMessages[i] = nextMessage;
    indexOfComma = nextIndex;
  }
}