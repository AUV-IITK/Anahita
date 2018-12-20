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
std::string task_recieved;
std::string current_task_str;


void taskCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Task being chaned to: [%s]", msg->data.c_str());
    for(int i = 0; i<3; i++)
        if(tasks[i].compare(msg->data.c_str()))
            current_task = i;
}

void changerCallback()
{
	current_task_str = tasks[current_task];
	if(task_recieved.compare(current_task_str)!=0)
	{
		std::cout<<"Task being recievd from param: " << task_recieved<<std::endl;
		std::cout<<"Current task from array: " << current_task_str <<std::endl;
		for(int i = 0; i<3; i++)
		{
			std::cout<<"Comparing with :" << tasks[i] << std::endl;	
        		if(tasks[i].compare(task_recieved)==0)
			{
			    std::cout<<"Comparsion successful"<<std::endl;
        		    current_task = i;
			    break;
			}
	
		}
		ROS_INFO("value of current task in mapping is %d", current_task);
	}
}

int main(int argc, char *argv[])
{    
    ros::init(argc, argv, "vision_node"); 
    //ros::Subscriber current_task_sub = nh.subscribe<std_msgs::String>("/current_task", 1000, taskCallback);
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Buoy buoy_;
    while(ros::ok)
    {
	nh.getParam("/current_task", task_recieved);
	changerCallback();
        if(previous_task != current_task)
        {
            if(current_task<3)
            {
		ROS_INFO("In inner loop");
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

