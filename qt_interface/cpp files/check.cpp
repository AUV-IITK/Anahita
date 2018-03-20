#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
using namespace std;
int main(int argc, char *argv[])
{
	ofstream myfile;
	

	ros::init(argc, argv, "master_test");
	ros::Time::init();
while(1){
	myfile.open("master_stat.txt");
	if (!ros::master::check())
	{
    		std::cout << "roscore inactive" << std::endl;
		std::cout << ros::master::check() << std::endl;
		myfile << "roscore inactive" << std::endl;
		myfile.close();
        	
    	}
	if (ros::master::check())
	{
    		std::cout << "roscore active" << std::endl;
    		myfile << "roscore active" << std::endl;
		std::cout << ros::master::check() << std::endl;
		myfile.close();
		
	}
        ros::Duration(1).sleep();

}
	return 0;
}	
