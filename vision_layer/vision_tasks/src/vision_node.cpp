#include "buoy.h"
#include "torpedo.h"
#include "line.h"
#include "markerDropper.h"
#include "ros/ros.h"

int main(int argc, char *argv[]){
    
    ros::init(argc, argv, "vision_node");    
    double input = atof(argv[1]);
    std::cout << input << std::endl;
    if(input == 1)
    {
        Buoy buoy_;
        buoy_.TaskHandling(true);           
        ros::Time next = ros::Time::now() + ros::Duration(10);
        int i = 0;
        while(1)
        {
            std::cout << "Changing the color" << std::endl;
            std::cout << "__________________________________________________________" << std::endl << std::endl;
            buoy_.switchColor(i+1);   
            while(ros::Time::now() < next){}
            next = ros::Time::now() + ros::Duration(10);
            i = (i+1)%3;
        }     
    }
    else if(input == 2)
    {
        Line line_;
        line_.TaskHandling();
    }
    else if(input== 3)
    {
        Torpedo torpedo_;
        torpedo_.TaskHandling();
    }
    else if(input == 4)
    {
        MarkerDropper markerdropper_;
        markerdropper_.BottomTaskHandling();
    }
    else
    {
        std::cerr << "Wrong Flags.\n Use 1 for Buoy.\n Use 2 for Line.\n Use 3 for Torpedo.\n Use 4 for Marker Dropper" << std::endl;
    }
}