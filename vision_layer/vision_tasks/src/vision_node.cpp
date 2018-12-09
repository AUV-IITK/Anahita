#include "buoy.h"
#include "torpedo.h"
#include "line.h"
#include "markerDropper.h"
#include "gate.h"
#include "ros/ros.h"
#include <string>
#include <std_msgs/String.h>

std::string tasks[3] = {"red_buoy", "yellow_buoy", "green_buoy"};
int current_task, previous_task;

void taskCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Task being chaned to: [%s]", msg->data.c_str());
    for(int i = 0; i<3; i++)
        if(tasks[i].compare(msg->data.c_str()))
            current_task = i;
}

int main(int argc, char *argv[])
{    
    ros::init(argc, argv, "vision_node");    
    ros::NodeHandle nh;
    ros::Subscriber current_task_sub = nh.subscribe<std_msgs::String>("/current_task", 1000,taskCallback);
  	ros::Rate loop_rate(10);

    while(ros::ok)
    {
        if(previous_task != current_task)
        {
            if(current_task<3)
            {
                Buoy buoy_;
                buoy_.switchColor(current_task);
                buoy_.TaskHandling(true);

            }
        }
        previous_task = current_task;    
        loop_rate.sleep();
	    ros::spinOnce();
    }
}


    // double input = atof(argv[1]);
    // std::cout << input << std::endl;
    // if(input == 1)
    // {
    //     Buoy buoy_;
    //     buoy_.TaskHandling(true);           
    //     ros::Time next = ros::Time::now() + ros::Duration(10);
    //     int i = 0;
    //     while(1)
    //     {
    //         // std::cout << "Changing the color" << std::endl;
    //         // std::cout << "__________________________________________________________" << std::endl << std::endl;
    //         // buoy_.switchColor(i+1);   
    //         // while(ros::Time::now() < next){}
    //         // next = ros::Time::now() + ros::Duration(10);
    //         // i = (i+1)%3;
    //     }     
    // }
    // else if(input == 2)
    // {
    //     Line line_;
    //     line_.TaskHandling();
    //     while(1){}        
    // }
    // else if(input== 3)
    // {
    //     Gate gate_;
    //     gate_.TaskHandling(true);
    //     while(1){}
    // }
    // else if(input== 4)
    // {
    //     Torpedo torpedo_;
    //     torpedo_.TaskHandling();
    //     while(1){}
    // }
    // else if(input == 5)
    // {
    //     MarkerDropper markerdropper_;
    //     markerdropper_.TaskHandling(true);
    //     while(1){}
    // }
    // //else if(input == 6)
    // //{
    // //    Octagon octagon_;
    //  //   octagon_.TaskHandling(true);
    //    // while(1){}
    // //}
    // else
    // {
    //     std::cerr << "Wrong Flags.\n Use 1 for Buoy.\n Use 2 for Line.\n Use 3 for Gate.\n Use 4 for Torpedo.\n Use 5 for Marker Dropper" << std::endl;
    // }

