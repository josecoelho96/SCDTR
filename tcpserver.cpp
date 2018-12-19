//
//  tcpserver.cpp
//  
//
//  Created by David Teles on 18/12/2018.
//

#include "tcpserver.hpp"
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <thread>
#include <time.h>


using namespace boost::asio;
using boost::system::error_code;

bool streamStatus = false;
long laststart;
char port[10];

class session {
    ip::tcp::socket s;
    enum { max_len = 1024 };
    char data[max_len];
    char msg[max_len];
    
    void hread(const error_code& ec, size_t sz) {
        std::cout << "Received:" << data;
        //Check the flags and send info
        switch (data[0]) {
            case 'g':
                switch (data[2]) {
                    case 'I':
                        std::cout << "Get current measured illuminance at desk <i>\n";
                        strcpy(msg,"I\n");
                        break;
                        
                    case 'd':
                        std::cout << "Get current duty cycle at luminaire i\n";
                        strcpy(msg,"d\n");
                        break;
                        
                    case 's':
                        std::cout << "Get current occupancy state at desk <i>\n";
                        strcpy(msg,"s\n");
                        break;
                        
                    case 'L':
                        std::cout << "Get current illuminance lower bound at desk <i>\n";
                        strcpy(msg,"L\n");
                        break;
                        
                    case 'o':
                        std::cout << "Get current external illuminance at desk <i>\n";
                        strcpy(msg,"o\n");
                        break;
                        
                    case 'r':
                        std::cout << "Get current illuminance control reference at desk <i>\n";
                        strcpy(msg,"r\n");
                        break;
                        
                    case 'p':
                        strcpy(msg,"p\n");
                        if (data[4] == 'T') {
                            std::cout << "Get instantaneous power consumption at desk <i>\n";
                        } else {
                            std::cout << "Get instantaneous total power consumption in  system.\n";
                        }
                        break;
                        
                    case 't':
                        std::cout << "Get elapsed time since last restart\n";
                        std::cout << "Elapsed time: " << (time(0) - laststart) <<'\n';
                        
                        sprintf(msg,"t %ld\n",(time(0) - laststart));
                        break;
                        
                    case 'e':
                        strcpy(msg,"e\n");
                        if (data[4] == 'T') {
                            std::cout << "Get accumulated energy consumption at desk <i> since the last system restart.\n";
                        } else {
                            std::cout << "Get total accumulated energy consumption since last system restart\n";
                        }
                        break;
                        
                    case 'c':
                        strcpy(msg,"c\n");
                        if (data[4] == 'T') {
                            std::cout << "Get accumulated comfort error at desk <i> since last system restart.\n";
                        } else {
                            std::cout << "Get total comfort error since last system restart.\n";
                        }
                        break;
                        
                    case 'v':
                        strcpy(msg,"v\n");
                        if (data[4] == 'T') {
                            std::cout << "Get accumulated comfort flicker at desk <i> since last system restart.\n";
                        } else {
                            std::cout << "Get total comfort flicker since last system restart.\n";
                        }
                        break;
                    default:
                        std::cout << "Bad command\n";
                        strcpy(msg, "Command not found!\n");
                        break;
                }
                break;
            case 'r':
                std::cout << "Restart system\n";
                strcpy(msg, "ack\n");
                laststart =  time(0);
                break;
                
            case 'b':
                std::cout << "Get last minute buffer of variable <x> of desk <i>\n";
                strcpy(msg, "ack\n");
                break;
                
            case 's':
                if(streamStatus == false){
                    streamStatus = true;
                    strcpy(msg, "Start Stream\n");
                } else {
                    streamStatus = false;
                    strcpy(msg, "Stop Stream\n");
                }
                break;
            default:
                strcpy(msg, "Command not found!\n");
                break;
        }
        
            //std::cout << "Message send:" << msg << "With size:" << strlen(msg) << '\n';
        if (!ec) async_write(s, buffer(msg, strlen(msg)),boost::bind(&session::hwrite, this, _1));
        else delete this;
    }
    
    void hwrite(const error_code& ec) {
        if (!ec) start();
        else delete this;
    }
    
public:
    session(io_service& io) : s(io) { }
    ip::tcp::socket& socket() {return s;}
    
    void start() {
        s.async_read_some(buffer(data,max_len),boost::bind(&session::hread, this, _1, _2));
        
    }
};


class server {
    io_service& io;
    ip::tcp::acceptor acc;
    
    void start_accept() {
        session* new_sess = new session(io);
        acc.async_accept(new_sess->socket(),boost::bind(&server::haccept, this, new_sess,_1));
    }
    
    void haccept(session* sess, const error_code& ec) {
        if (!ec) sess->start();
        else delete sess;
        start_accept();
    }
    
    
public:
    server(io_service& io, short port): io(io), acc(io, ip::tcp::endpoint(ip::tcp::v4(), port)) {
        
        start_accept();
    }
};

void startserver(){
    std::cout<< "Server Thread created!" << '\n';
    io_service io;
    server s(io, std::atoi(port));
    std::cout<< "Server running in port:" << port << "with IP:" << ip::address_v4() << '\n';
    io.run();
}


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: server <port>\n"; return 1; }
    std::cout<< "Starting!" << '\n';
    strcpy(port,argv[1]);
    laststart =  time(0);
    std::thread tcpserver(startserver);
    //std::thread i2c{};
    tcpserver.join();
    //i2c.join();
    std::cout<< "Finished" << '\n';
}

