
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <MS5837.h>

#define servoEastPin 27       // pin definitions for forward thrusters
#define servoWestPin 35

#define servoNorthSwayPin 31   // pin definitions for sideward thrusters
#define servoSouthSwayPin 23
//31 30 32 33
#define servoNorthWestUpPin 29// pin definitions for upward thrusters
#define servoSouthWestUpPin 33
#define servoNorthEastUpPin 25 
#define servoSouthEastUpPin 37

Servo servoEast;
Servo servoWest;
Servo servoNorthSway;
Servo servoSouthSway;
Servo servoNorthWestUp;
Servo servoNorthEastUp;
Servo servoSouthWestUp;
Servo servoSouthEastUp;
Servo servoMarkerDropper;
Servo led;

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
double batt_voltage;

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

// function declration to puboish pressure sensor data
void publish_pressure_data();

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
//    while(!Serial)
//    {
//      // Waiting for a connection to be generated, needed for delays caused by USB
//    }
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

    led.attach(7);
    led.writeMicroseconds(1100);
  
    servoNorthWestUp.attach(servoNorthWestUpPin);
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
    servoNorthEastUp.attach(servoNorthEastUpPin);
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
    servoSouthWestUp.attach(servoSouthWestUpPin);
    servoSouthWestUp.writeMicroseconds(ESC_Zero);
    servoSouthEastUp.attach(servoSouthEastUpPin);
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
    delay(7000);
   
    //
    // setting up pressure sensor
    Wire.begin();
    // We can't continue with the rest of the program unless we can initialize the sensor
    while(!pressure_sensor.init())
    {
        //Serial.println("Pressure sensor failed");
    }
    pressure_sensor.setModel(MS5837::MS5837_30BA);
    pressure_sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)*/   
}

void loop()
{
    static unsigned long prev_pressure_time = 0;

     //this block publishes the pressure sensor data based on defined rate
    if ((millis() - prev_pressure_time) >= (1000 / PRESSURE_PUBLISH_RATE))
    {
        // publish_pressure_data();
        prev_pressure_time = millis();
    }
    
    delay(10);
    //Just calling the two loops which will handle i/o as well
    actuatorLoop();
    sensorLoop();
    led.writeMicroseconds(1100);
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
    TEast(-splitMessages[3].toInt());
    TWest(-splitMessages[2].toInt());
    TNorth(-splitMessages[0].toInt());
    TSouth(-splitMessages[1].toInt());
    TNEUp(-splitMessages[4].toInt());
    TNWUp(-splitMessages[5].toInt());
    TSEUp(-splitMessages[6].toInt());
    TSWUp(-splitMessages[7].toInt());
  }
}
  

void sensorLoop(){

  //Using the idea of printing data in CSV format
  //
  read_pressure_data();
  Serial.print(batt_voltage);
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
    depth_reading = pressure_sensor.depth(); //convert to centimeters

    batt_voltage = analogRead(A0)*125/1024;
    
}

void TEast(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoEast.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoEast.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoEast.writeMicroseconds(ESC_Zero);
}
void TWest(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoWest.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoWest.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoWest.writeMicroseconds(ESC_Zero);
}
void TNorth(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoNorthSway.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthSway.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthSway.writeMicroseconds(ESC_Zero);
}
void TSouth(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoSouthSway.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthSway.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthSway.writeMicroseconds(ESC_Zero);
}
void TNEUp(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthEastUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthEastUp.writeMicroseconds(ESC_Zero);
}
void TNWUp(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoNorthWestUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoNorthWestUp.writeMicroseconds(ESC_Zero);
}
void TSEUp(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
  if(data>0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + data + deadZone);
  else if(data<0)
    servoSouthEastUp.writeMicroseconds(ESC_Zero + data - deadZone);
  else
    servoSouthEastUp.writeMicroseconds(ESC_Zero);
}
void TSWUp(int data)
{
  if(data>125) data=125;
  if(data<-125) data=-125;
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
