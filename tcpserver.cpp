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

float lerfloat(){
    return 0.0;
}

int lerint(){
    return 0;
}

bool lerbool(){
    return true;
}

long lerlong(){
    return 0;
}

void getvalues(char* temp){
    temp[0]='1';
    temp[1]=';';
    temp[2]='2';
    temp[3]='\n';
}

void resetvalues(){
    
}


class session {
    ip::tcp::socket s;
    enum { max_len = 1024 };
    char data[max_len];
    char msg[max_len];
    char bbuffer[1000];
    
    
    void hread(const error_code& ec, size_t sz) {
        std::cout << "Received:" << data;
        //Temporary variables
        char a,b;
        int c;
        //Check the flags and send info
        switch (data[0]) {
            case 'g':
                switch (data[2]) {
                    //DONE (Add correct function)
                    case 'I':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current measured illuminance at desk " << c << ".\n";
                        sprintf(msg,"I %d %f\n",c, lerfloat());
                        break;
                    //DONE (Add correct function)
                    case 'd':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current duty cycle at luminaire " << c << ".\n";
                        sprintf(msg,"d %d %f\n",c, lerfloat());
                        break;
                    //DONE (Add correct function)
                    case 's':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current occupancy state at desk " << c << ".\n";
                        sprintf(msg,"s %d %d\n",c, lerbool());
                        break;
                    //DONE (Add correct function)
                    case 'L':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current illuminance lower bound at desk " << c << ".\n";
                        sprintf(msg,"L %d %f\n",c, lerfloat());
                        break;
                    //DONE (Add correct function)
                    case 'o':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current external illuminance at desk " << c << ".\n";
                        sprintf(msg,"o %d %f\n",c, lerfloat());
                        break;
                    //DONE (Add correct function)
                    case 'r':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current illuminance control reference at desk " << c<< ".\n";
                        sprintf(msg,"r %d %f\n",c, lerfloat());
                        break;
                    //DONE (Add correct function)
                    case 'p':
                        if (data[4] == 'T') {
                            std::cout << "Get instantaneous total power consumption in  system.\n";
                            sprintf(msg,"p T %f\n",lerfloat());
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get instantaneous power consumption at desk " << c << ".\n";
                            sprintf(msg,"p %d %f\n",c, lerfloat());
                        }
                        break;
                    //DONE
                    case 't':
                        std::cout << "Get elapsed time since last restart.\n";
                        std::cout << "Elapsed time: " << (time(0) - laststart) <<'\n';
                        
                        sprintf(msg,"t %ld\n",(time(0) - laststart));
                        break;
                    //DONE (Add correct function)
                    case 'e':
                        if (data[4] == 'T') {
                            std::cout << "Get total accumulated energy consumption since last system restart.\n";
                            sprintf(msg,"e T %f\n",lerfloat());
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get accumulated energy consumption at desk " << c << " since the last system restart.\n";
                            sprintf(msg,"e %d %f\n",c, lerfloat());
                            
                        }
                        break;
                    //DONE (Add correct function)
                    case 'c':
                        if (data[4] == 'T') {
                            std::cout << "Get total comfort error since last system restart.\n";
                            sprintf(msg,"c T %f\n",lerfloat());
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get accumulated comfort error at desk " <<c << " since last system restart.\n";
                            sprintf(msg,"c %d %f\n",c, lerfloat());
                            
                        }
                        break;
                    //DONE (Add correct function)
                    case 'v':
                        strcpy(msg,"v\n");
                        if (data[4] == 'T') {
                            std::cout << "Get total comfort flicker since last system restart.\n";
                            sprintf(msg,"v T %f\n",lerfloat());
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get accumulated comfort flicker at desk <i> since last system restart.\n";
                            sprintf(msg,"v %d %f\n",c, lerfloat());
                        }
                        break;
                    default:
                        std::cout << "Bad command\n";
                        strcpy(msg, "Command not found!\n");
                        break;
                }
                break;
            //DONE (Add correct function)
            case 'r':
                std::cout << "Restart system\n";
                strcpy(msg, "ack\n");
                laststart =  time(0);
                resetvalues();
                break;
            //DONE (Add correct function)
            case 'b':
                sscanf(data,"%c %c %d",&a, &b, &c);
                std::cout << "Get last minute buffer of variable " << b << " of desk " << c << ".\n";
                if (b == 'I') {
                    getvalues(bbuffer);
                    sprintf(msg,"b %c %d %s\n",b ,c, bbuffer);
                } else if (b == 'd') {
                    getvalues(bbuffer);
                    sprintf(msg,"b %c %d %s\n",b ,c, bbuffer);
                } else {
                    sprintf(msg,"b %c isn't a valid command\n",b);
                }
                
                
                
                break;
                
            case 's':
                sscanf(data,"%c %c %d",&a, &b, &c);
                if(streamStatus == false){
                    streamStatus = true;
                    std::cout << "Start stream of variable " << b << " of desk " << c << ".\n";
                    sprintf(msg,"s %c %d %f %ld\n",b ,c , lerfloat(), lerlong());

                } else {
                    std::cout << "Stop stream of variable " << b << " of desk " << c << ".\n";
                    streamStatus = false;
                    strcpy(msg, "ack\n");
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
    std::cout<< "Server running in port:" << port << " with IP:" << ip::address_v4() << '\n';
    io.run();
}

void starti2c(){
    std::cout<< "I2C Thread started!" << '\n';
    while (1) {
        std::cout<< "Running..." << '\n';
        usleep(100000);
    }
    
    
}

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: server <port>\n"; return 1; }
    std::cout<< "Starting!" << '\n';
    strcpy(port,argv[1]);
    laststart =  time(0);
    std::thread tcpserver(startserver);
    std::thread i2c(starti2c);
    tcpserver.join();
    i2c.join();
    std::cout<< "Finished" << '\n';
}

