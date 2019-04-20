#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
// #include <anahita_msgs/Depth.h>
// #include <anahita_msgs/Pressure.h>
#include <anahita_msgs/Thrust.h>

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

void PWMCb(const anahita_msgs::Thrust& msg_);
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

int ESC_Zero = 1500;
int deadZone = 25;
int torpedo_count = 0;
ros::NodeHandle nh;

// define rate at which sensor data should be published (in Hz)
#define PRESSURE_PUBLISH_RATE 10

// pressure sensor
MS5837 pressure_sensor;

// declare subscribers
ros::Subscriber<anahita_msgs::Thrust> PWM_Sub("/pwm", &PWMCb);

// function declration to puboish pressure sensor data
void publish_pressure_data();

// declaration of callback functions for all thrusters
// declare publishers

// ros::Subscriber<std_msgs::Int32> Marker_Dropper_Sub("/marker_dropper", &MDCb);

// ros::Subscriber<std_msgs::Int32> SOV_Sub("/torpedo", &SOVCb);

std_msgs::Float32 depth_msg;
ros::Publisher ps_depth_pub("/pressure_sensor/depth", &depth_msg);

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

void setup()
{  
    Serial.begin(57600);

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
    Serial.print("Successfully started servos");    
    
    // setting up pressure sensor
     Wire.begin();
    // We can't continue with the rest of the program unless we can initialize the sensor
     pressure_sensor.init();
     pressure_sensor.setFluidDensity(997);    //kg/m^3 (freshwater, 1029 for seawater)*/
    
    // initialize ROS node
    nh.initNode();
    nh.getHardware()->setBaud(57600);

    // start ros_node
    nh.subscribe(PWM_Sub);
    // nh.subscribe(Marker_Dropper_Sub);
    // nh.subscribe(SOV_Sub);
    
    // publisher
     nh.advertise(ps_depth_pub);
    
    while (!nh.connected())
    {
         nh.spinOnce();
    }
    nh.loginfo("Anahita CONNECTED");
    
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
    
    //nh.loginfo("Data being recieved");
    delay(10);
    
    TEast(forward_right);
    TWest(forward_left);
    TNorth(sideward_front);
    TSouth(sideward_back);
    TNEUp(upward_north_east);
    TNWUp(upward_north_west);
    TSEUp(upward_south_east);
    TSWUp(upward_south_west);
    MDCb(marker_dropper);
    SOVCb(torpedo);

    if(torpedo_count == 0)
    {
      digitalWrite(TorpedoPin1 ,LOW);
      digitalWrite(TorpedoPin2 ,LOW);
    }
    nh.spinOnce();
    digitalWrite(TorpedoPin1, HIGH);
}

// function definition for publish_pressure_data
void publish_pressure_data()
{
    pressure_sensor.read();
   // pressure_msg.data = pressure_sensor.pressure();

    /** Depth returned in meters (valid for operation in incompressible
    *  liquids only. Uses density that is set for fresh or seawater.
    */
    depth_msg.data = -100*pressure_sensor.depth(); //convert to centimeters

    ps_depth_pub.publish(&depth_msg);
}

void MDCb(const int msg)
{
  nh.loginfo("Inside marker dropper callback");
  int data = msg;
  int pos = 0;
  if(data == 1)
  {
    nh.loginfo("First ball being dropped");
    for(pos = 0; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
  }
  if(data == 2)
  {
    nh.loginfo("Second ball being dropped");
    for(pos = 170; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
  }
 if(data == -1)
 { 
   nh.loginfo("Can now load the inner ball");
   for(pos = 0; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
 }
 if(data == -2)
 {
   nh.loginfo("Can now load the outer ball");
   for(pos = 0; pos < 170; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servoMarkerDropper.write(pos);              // tell servo to go to position in variable 'pos' 
      delay(15);                       // waits 15ms for the servo to reach the position 
    }
 }
}

void SOVCb(const int msg)
{
  nh.loginfo("Inside SOV callback");
  int data = msg;
  int pos = 0;
  if(data == 1)
  {
    torpedo_count = 1;
    nh.loginfo("First Torpedo being shot");
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

void PWMCb(const anahita_msgs::Thrust& msg_)
{
    // nh.loginfo("Inside PWM Callback");
    forward_right = msg_.forward_right;
    forward_left = msg_.forward_left;
    sideward_front = msg_.sideward_front;
    sideward_back = msg_.sideward_back;
    upward_north_east = msg_.upward_north_east;
    upward_north_west = msg_.upward_north_west;
    upward_south_east = msg_.upward_south_east;
    upward_south_west = msg_.upward_south_west;
    marker_dropper = msg_.marker_dropper;
    torpedo = msg_.torpedo;
}
