#include <dvl_ethernet.h>

DVLEthernet::DVLEthernet() {}
DVLEthernet::~DVLEthernet() {}
std::string dataToSend = "CS\n";
uint32_t dataLength = htonl(dataToSend.size()); // Ensure network byte order 

void DVLEthernet::Connect(std::string address, int port)
{
    struct sockaddr_in server;

    socket_ = socket(AF_INET, SOCK_STREAM, 0);

    if(socket_ == -1)
        ROS_INFO("Could not create the socket for DVL");
    
    //filling up the server variable using information we already know
    server.sin_addr.s_addr = inet_addr(address.c_str());
    server.sin_family = AF_INET;
    server.sin_port = htons(port);    

    if (connect(socket_, (struct sockaddr *) &server, sizeof(server)) < 0) {
        ROS_INFO("Connection error with the DVL");
        return;
    }

    ROS_INFO("Connected\n");
    send(socket_,dataToSend.c_str(),dataToSend.size(),MSG_CONFIRM); // Send the string 

}   

void DVLEthernet::Receive() {


    if (recv(socket_, data_, 2048, 0) < 0)
    {
        ROS_INFO("Recieve failed from the DVL");
    }
    ROS_INFO("Reply recieved from the DVL");
}

char* DVLEthernet::GetRawData() {
    return data_;
}