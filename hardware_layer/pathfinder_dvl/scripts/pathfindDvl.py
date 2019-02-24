#CALIFORNIA STATE UNIVERSITY TELEDYNE PATHFINDER DVL DRIVERS - ROBOSUB 2018# 
########### by Diego Santillan in collaboration with Adam Loeffler###########
################### Additional ROS work by Jonathan Song ####################

import rospy
from pathfinder_dvl.msg import DVL
#from ez_async_data.msg import Rotation
from std_msgs.msg import Float32
import serial
import time

class RunDVL:
    def __init__(self):
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
    # self.yaw will always only have a length of 1 max to keep most updated
    # imu is 0 degrees west 90 deg north -90 deg south +- 180 east
    # dvl takes 0 - 359.99 with 0 deg north, 90 deg east, 180 south, and 270 west
    # this is assuming typical nautical heading
    def rCallBack(self, rotation):
        if 90 <= rotation.yaw and rotation.yaw <= 180:
            heading = rotation.yaw - 90
        else:
            heading = rotation.yaw + 270
        # logic for else :
        # elif -180 <= rotation.yaw and rotation.yaw <= -90:
        #     heading = rotation.yaw + 270
        # elif -90 < rotation.yaw and rotation.yaw < 0:
        #     heading = rotation.yaw + 270
        # elif 0 <= rotation.yaw and rotation.yaw < 90:
        #     heading = rotation.yaw + 270
        self.yaw = heading
        self.pitch = rotation.pitch
        self.roll = rotation.roll
        
        # print('current yaw: %.2f' % self.yaw)

    def main(self):

        #########################INITALIZE DVL SERIAL################################

        #dvl = serial.Serial("COM13", 115200) #Windows Serial
        dvl = serial.Serial("/dev/ttyUSB0", 115200) #Ubuntu Serial
        #dvl = serial.Serial("/dev/ttyUSB1", 115200) #Ubuntu Serial
        #dvl = serial.Serial("/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0", 115200) #Ubuntu Serial
        rospy.init_node('dvl_node', anonymous=True)


        ################PATHFINDER DVL COMMANDS TO STREAM DATA#####################

        dvl.write("===") #DVL Break (PathFinder Guide p. 24 and p.99)

        rospy.sleep(5) #sleep for 2 seconds

        # ROS publisher setup
        pub = rospy.Publisher('dvl_status', DVL, queue_size = 1)
        # pubHeading = rospy.Publisher('dvl_heading', Float32, queue_size = 1)
        pubSS = rospy.Publisher('dvl_ss', Float32, queue_size = 1)
        #rospy.Subscriber('current_rotation', Rotation, self.rCallBack, queue_size = 1)
        msg = DVL()
        # msgHeading = Float32()
        msgSS = Float32()
        
        #PD6 settings --------------------------------------------------------------
        dvl.write("CR1\r") #set factory defaults.(Pathfinder guide p.67)
        dvl.write("CP1\r") # required command
        #PD6 settings --------------------------------------------------------------
        dvl.write("PD0\r") #pd6 data format (Pathfinder Guide p.207) <---important
        # dvl.write("BK0\r")
        #PD13 settings -------------------------------------------------------------
        # dvl.write("PD13\r")
        dvl.write("EX11110\r") #coordinate transformation (Pathfinder guide p.124)
        dvl.write("EA+4500\r") #heading alignment (Pathfinder guide 118)
        # dvl.write("EZ11110010\r") #internal speed of sound, depth, heading, pitch, roll, temperature
        dvl.write("EZ11000010\r") #internal speed of sound, depth, temperature
        # dvl.write("EZ10000010\r") #default sensor source (Pathfinder guide 125)
        # dvl.write("EZ11011010\r") #internal speed of sound, depth, pitch, roll, temperature
        # dvl.write("EZ10000010\r") #internal speed of sound, temperature

        dvl.write("CK\r") #stores present parameters (Pathfinder guide 114)
        dvl.write("CS\r") #start pinning (Pathfinder guide 115)

        #NOTE: the \r character is required for continuous stream i.e (PD6\r")

        ################################PROGRAM BEGINS##############################
        print "dvl start"
        heading = 0 
        dvl_heading = 0
        east_trans = 0
        north_trans = 0
        up_trans = 0
        east_vel = 0
        north_vel = 0
        up_vel = 0
        status = 0

        pitch = 0
        roll = 0
        heading = 0

        loop_time = time.time()
        pubTimePrev = loop_time
        # pubTimeInterval = 0.01
        heading_time_prev = loop_time
        heading_time_interval = 0.014
        mod_val = 0
        while not rospy.is_shutdown():
            loop_time = time.time()

            if loop_time - heading_time_prev > heading_time_interval:
                heading_time_prev = loop_time
                #and mod_val % 2 == 0
                if self.yaw and mod_val % 2 == 0:
                    mod_val = 1
                    heading = self.yaw #put heading info ** heree from IMU
                    heading *= 100 #heading needs to go from 0 to 35999 (see Heading Alignment Pathfinder p.118)
                    heading_int = int(heading)
                    if (heading - heading_int) >= 0.5:
                        heading_int += 1
                    dvl.write("EH " + str(heading_int) + ", 1\r") #Update Heading
                if self.pitch and self.roll and mod_val % 2 == 1:
                    mod_val = 0
                    pitch = self.pitch 
                    pitch *= 100 
                    pitch_int = int(pitch)
                    if (pitch - pitch_int) >= 0.5:
                        pitch_int += 1

                    roll = self.roll
                    roll *= 100
                    roll_int = int(roll)
                    if (roll - roll_int) >= 0.5:
                        roll_int += 1

                    dvl.write("EP " + str(pitch_int) + ", " + str(roll_int) + ", 1\r") #Update Heading
            print("in loop")
            if dvl.in_waiting > 0: #If there is a message from the DVL
                try:
                    line = dvl.readline()
                except:
                    print("read fail")
# 
                # print(line)

                if line[:3] == ":BD": #If the message is a positional update
                    line = line.split(",")
                    # north_trans = float(line[1])
                    # east_trans = float(line[2])
                    east_trans = float(line[1])
                    north_trans = float(line[2])
                    up_trans = float(line[3])
                    rangeToBottom = float(line[4])
                    timeDifference = float(line[5])
                    #setup msg to be published to ROS
                    msg.xpos = east_trans
                    msg.ypos = north_trans
                    msg.zpos = up_trans
                    pub.publish(msg)

                elif line[:3] == ":BS": #If the message is a velocity update
                    line = line.split(",")
                    port_vel = float(line[1])
                    aft_vel = float(line[2])
                    up_vel = float(line[3])
                    status = line[4]
                    msg.xvel = port_vel #need to change msg var names
                    msg.yvel = aft_vel
                    msg.zvel = up_vel
                    pub.publish(msg)

                # elif line[:3] == ":SA": #If the message is orientation 
                #     line = line.split(",")
                #     # pitch = float(line[1])
                #     # roll = float(line[2])
                #     print(line[0])
                #     print(line[1])
                #     print(line[2])
                #     print(line[3])
                #     dvl_heading = float(line[3])
                #     msgHeading.data = dvl_heading
                #     pubHeading.publish(msgHeading)

                elif line[:3] == ":TS": #If the message is a timestamp
                    line = line.split(",")
                    msgSS.data = float(line[5])
                    pubSS.publish(msgSS)
                    
                    # print line
                    # print "Heading:", heading, "east_vel:", east_vel, "north_vel:", north_vel, "depth_vel:", depth_vel, "Status:", status, "\r"
                    
                # print "IMU Heading:", heading, "DVL Heading:", dvl_heading, "east_vel:", east_vel, "north_vel:", north_vel, "depth_vel:", depth_vel, "Status:", status, "Xpos", east_trans, "YPos", north_trans, "ZPos", depth_trans
                
                # if (loop_time - pubTimePrev) > pubTimeInterval:
                #     pubTimePrev = loop_time


dvl = RunDVL()

if __name__ == '__main__':
    try:
        dvl.main()
    except rospy.ROSInterruptException: 
        pass
