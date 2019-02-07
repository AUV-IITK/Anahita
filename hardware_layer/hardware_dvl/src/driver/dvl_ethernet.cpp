#include <dvl_ethernet.h>

DVLEthernet::DVLEthernet() {}
DVLEthernet::~DVLEthernet() {}

void DVLEthernet::Connect(std::string address, int port)
{
    struct sockaddr_in server;

    socket_ = socket(AF_INET, SOCK_STREAM, 0);

    if(socket_ == -1)
        ROS_DEBUG("Could not create the socket for DVL");
    
    //filling up the server variable using information we already know
    server.sin_addr.s_addr = inet_addr(address.c_str());
    server.sin_family = AF_INET;
    server.sin_port = htons(port);    

    if (connect(socket_, (struct sockaddr *) &server, sizeof(server)) < 0) {
        ROS_DEBUG("Connection error with the DVL");
        return;
    }

    ROS_DEBUG("Connected\n");
}   

void DVLEthernet::Receive() {
    if (recv(socket_, data_, 2048, 0) < 0)
    {
        ROS_DEBUG("Recieve failed from the DVL");
    }
    ROS_DEBUG("Reply recieved from the DVL");
}

char* DVLEthernet::GetRawData() {
    return data_;
}