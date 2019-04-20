#ifndef DVL_ETHERNET_H
#define DVL_ETHERNET_H

#include <string>
#include <ros/ros.h>
#include <sys/socket.h>
#include <arpa/inet.h>              // definitions for internet operations

class DVLEthernet {
    public:

        DVLEthernet();
        ~DVLEthernet();

        void Connect(std::string address, int port);
        void Receive();
        char* GetRawData();

    private:
  
        int socket_;
        char data_[2048];

};

#endif //DVL_ETHERNET_H