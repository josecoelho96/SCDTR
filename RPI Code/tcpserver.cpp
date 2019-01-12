
//=======================SERVER===============================
#include "tcpserver.hpp"
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <thread>
#include <mutex>
#include <time.h>
//=======================SERVER===============================

//=======================DATA=================================
#include <list>
#include "Desk.h"

std::mutex mtx;

std::list<Desk> lista;
std::list<Desk>::iterator it;

Desk findID(int id){
    
    //std::cout << "Searching for ID: " << id <<'\n';
    for (it = lista.begin(); it != lista.end(); ++it) {
        //std::cout << "ID: " << it->getID() << '\n';
        if (it->getID() == id) {
            break;
        }
    }
    
    return *it;
}
//=======================DATA=================================

//=======================I2C==================================
#include <stdio.h>
#include <pigpio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <memory.h>
#include "MessageTypes.h"
#define DESTINATION_ADDR 0x00
#define SLAVE_ADDR 0x00


int close_slave(bsc_xfer_t & xfer);
int init_slave(bsc_xfer_t &xfer, int addr);
int starti2c();
float bytestofloat(unsigned char a,unsigned char b,unsigned char c, unsigned char d);


int status, j;
int key = 0;
int handle;
bsc_xfer_t xfer;
volatile int terminate = 0;
//=======================I2C==================================

//=======================RANDOM===============================



//for comand r reset all values
void resetvalues(){
    for (it = lista.begin(); it != lista.end(); ++it) {
        it->reset();
    }
}

//=======================RANDOM===============================

//=======================SERVER===============================

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
    char bbuffer[1000];
    
    
    void hread(const error_code& ec, size_t sz) {
        std::cout << "Received:" << data;
        //Temporary variables
        char a,b;
        int c;
        float sum = 0.0;
        //Check the flags and send info
        switch (data[0]) {
            case 'g':
                switch (data[2]) {
                    //DONE (Add correct function)
                    case 'l':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current measured illuminance at desk " << c << ".\n";
                        mtx.lock();
                        findID(c);
                        if(it != lista.end()){
                            sprintf(msg,"l %d %f\n",c, it->getIluminance());
                        } else {
                            sprintf(msg,"l %d luminaire not found!\n",c);
                        }
                        mtx.unlock();
                        
                        break;
                    //DONE (Add correct function)
                    case 'd':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current duty cycle at luminaire " << c << ".\n";
                        mtx.lock();
                        findID(c);
                        if(it != lista.end()){
                            sprintf(msg,"d %d %f\n",c, it->getDutyCicle());
                        } else {
                            sprintf(msg,"d %d luminaire not found!\n",c);
                        }
                        mtx.unlock();
                        
                        break;
                    //DONE (Add correct function)
                    case 's':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current occupancy state at desk " << c << ".\n";
                        mtx.lock();
                        findID(c);
                        if(it != lista.end()){
                            sprintf(msg,"s %d %d\n",c, it->getOccupancyState());
                        } else {
                            sprintf(msg,"s %d luminaire not found!\n",c);
                        }
                        mtx.unlock();
                        break;
                    //DONE (Add correct function)
                    case 'L':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current illuminance lower bound at desk " << c << ".\n";
                        mtx.lock();
                        findID(c);
                        if(it != lista.end()){
                            sprintf(msg,"L %d %f\n",c, it->getil_LowerBound());
                        } else {
                            sprintf(msg,"L %d luminaire not found!\n",c);
                        }
                        mtx.unlock();
                        break;
                    //DONE (Add correct function)
                    case 'o':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current external illuminance at desk " << c << ".\n";
                        mtx.lock();
                        findID(c);
                        if(it != lista.end()){
                            sprintf(msg,"o %d %f\n",c, it->getil_External());
                        } else {
                            sprintf(msg,"o %d luminaire not found!\n",c);
                        }
                        mtx.unlock();
                        break;
                    //DONE (Add correct function)
                    case 'r':
                        sscanf(data,"%c %c %d",&a, &b, &c);
                        std::cout << "Get current illuminance control reference at desk " << c<< ".\n";
                        mtx.lock();
                        findID(c);
                        if(it != lista.end()){
                            sprintf(msg,"r %d %f\n",c, it->getControlRef().getValue());
                        } else {
                            sprintf(msg,"r %d luminaire not found!\n",c);
                        }
                        mtx.unlock();
                        break;
                    //DONE (Add correct function)
                    case 'p':
                        if (data[4] == 'T') {
                            sum = 0.0;
                            std::cout << "Get instantaneous total power consumption in  system.\n";
                            mtx.lock();
                            for (it = lista.begin(); it != lista.end(); ++it) {
                                sum+=it->getDimming().getValue();
                            }
                            mtx.unlock();
                            sprintf(msg,"p T %f\n",sum);
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get instantaneous power consumption at desk " << c << ".\n";
                            mtx.lock();
                            findID(c);
                            if(it != lista.end()){
                                sprintf(msg,"p %d %f\n",c, it->getDimming().getValue());
                            } else {
                                sprintf(msg,"p %d luminaire not found!\n",c);
                            }
                            mtx.unlock();
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
                            sum = 0.0;
                            std::cout << "Get total accumulated energy consumption since last system restart.\n";
                            mtx.lock();
                            for (it = lista.begin(); it != lista.end(); ++it) {
                                it->setEnergy();
                                sum+=it->getEnergy();
                            }
                            mtx.unlock();
                            sprintf(msg,"e T %f\n",sum);
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get accumulated energy consumption at desk " << c << " since the last system restart.\n";
                            mtx.lock();
                            findID(c);
                            if(it != lista.end()){
                                
								sprintf(msg,"e %d %f\n",c, it->getEnergy());
							} else {
								sprintf(msg,"e %d luminaire not found!\n",c);
							}
                            mtx.unlock();
                        }
                        break;
                    //DONE (Add correct function)
                    case 'c':
                        if (data[4] == 'T') {
                            sum = 0.0;
                            std::cout << "Get total comfort error since last system restart.\n";
                            mtx.lock();
                            for (it = lista.begin(); it != lista.end(); ++it) {
                                it->setConfortError();
                                sum+=it->getConfortError();
                            }
                            mtx.unlock();
                            sprintf(msg,"c T %f\n",sum);
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get accumulated comfort error at desk " <<c << " since last system restart.\n";
                            mtx.lock();
                            findID(c);
                            if(it != lista.end()){
                                it->setConfortError();
                                sprintf(msg,"c %d %f\n",c, it->getConfortError());
                            } else {
                                sprintf(msg,"c %d luminaire not found!\n",c);
                            }
                            mtx.unlock();
                            
                        }
                        break;
                    //DONE (Add correct function)
                    case 'v':
                        strcpy(msg,"v\n");
                        if (data[4] == 'T') {
                            sum = 0.0;
                            std::cout << "Get total comfort flicker since last system restart.\n";
                            mtx.lock();
                            for (it = lista.begin(); it != lista.end(); ++it) {
                                it->setConfortFlicker();
                                sum+=it->getConfortFlicker();
                            }
                            mtx.unlock();
                            sprintf(msg,"v T %f\n",sum);
                        } else {
                            sscanf(data,"%c %c %d",&a, &b, &c);
                            std::cout << "Get accumulated comfort flicker at desk <i> since last system restart.\n";
                            mtx.lock();
                            findID(c);
                            if(it != lista.end()){
                                it->setConfortFlicker();
                                sprintf(msg,"v %d %f\n",c, it->getConfortFlicker());
                            } else {
                                sprintf(msg,"v %d luminaire not found!\n",c);
                            }
                            mtx.unlock();
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
                if (b == 'l') {
                    mtx.lock();
                    findID(c);
                    if(it != lista.end()){
                        it->getLastLuminance(bbuffer);
                        sprintf(msg,"b %c %d %s\n",b ,c, bbuffer);
                    } else {
                        sprintf(msg,"b %c %d not found\n",b ,c);
                    }
                    mtx.unlock();
                    
                } else if (b == 'd') {
                    mtx.lock();
                    findID(c);
                    if(it != lista.end()){
                        it->getLastDimming(bbuffer);
                        
                        sprintf(msg,"b %c %d %s\n",b ,c, bbuffer);
                    } else {
                        sprintf(msg,"b %c %d not found\n",b ,c);
                    }
                    mtx.unlock();
                } else {
					sprintf(msg,"b %c isn't a valid command\n",b);
				}
                
                
                
                break;
                
            case 's':
                sscanf(data,"%c %c %d",&a, &b, &c);
                if(streamStatus == false){
                    streamStatus = true;
                    std::cout << "Start stream of variable " << b << " of desk " << c << ".\n";
                    sprintf(msg,"s %c %d %f %ld\n",b ,c , 0.0, time(0));

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
        
//=======================SERVER===============================
//=======================I2C==================================
int starti2c(){
	std::cout<< "I2C Thread started!" << '\n';
    
    int length = 12; //11 chars + \0
    char *message;
    
    if (gpioInitialise() < 0) {
        printf("Erro 1\n");
        return 1;
    }
    
    

	status = init_slave(xfer, SLAVE_ADDR);
	handle = i2cOpen(1, DESTINATION_ADDR, 0); /* Initialize */
    Desk *aux;
    
    while(terminate == 0) {
        xfer.txCnt = 0;
        status = bscXfer(&xfer);
        //Message needs to have a minimum of 2 bytes
        if (xfer.rxCnt > 1) {

            switch (xfer.rxBuf[1]) {
				case MT_OK:
                    printf("MT_OK from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_WAIT:
                    printf("MT_WAIT from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_ALONE:
                    printf("MT_ALONE from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_NETWORK:
                    printf("MT_NETWORK from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_CALIBRATION_VALUE_AFFECTED:
					if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
						printf("MT_CALIBRATION_VALUE_AFFECTED  from %d = %f\n",xfer.rxBuf[0], bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                    } else {
						printf("Received %d non identified bytes\n", xfer.rxCnt);
						for (j = 0; j < xfer.rxCnt; j++) {
							printf("%d",xfer.rxBuf[j]);
						}
					}
                    break;
                case MT_CALIBRATION_VALUE_OWN:
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        printf("MT_CALIBRATION_VALUE_OWN  from %d = %f\n",xfer.rxBuf[0], bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                    } else {
                        printf("Received %d non identified bytes\n", xfer.rxCnt);
                        for (j = 0; j < xfer.rxCnt; j++) {
                            printf("%d",xfer.rxBuf[j]);
                        }
                    }
                    break;
                case MT_STATE:
                    printf("MT_STATE from %d\n",xfer.rxBuf[0]);
                    if(xfer.rxCnt > 3){
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setOccupancyState((bool)xfer.rxBuf[3]);
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_BRIGHTNESS:
                    printf("MT_BRIGHTNESS from %d\n",xfer.rxBuf[0]);
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setDutyCicle(bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_LUX:
                    printf("MT_LUX from %d = %f\n",xfer.rxBuf[0],bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setIluminance(bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_LOWER_BOUND:
                    printf("MT_LOWER_BOUND from %d\n",xfer.rxBuf[0]);
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setil_LowerBound(bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_EXTERNAL:
                    printf("MT_EXTERNAL from %d\n",xfer.rxBuf[0]);
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setil_External(bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_CONTROLLER_REF:
                    printf("MT_CONTROLLER_REF from %d\n",xfer.rxBuf[0]);
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setControlRef(bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_DIMMING:
                    
                    if (xfer.rxCnt > 5 && xfer.rxBuf[2]==8) {
                        printf("MT_DIMMING from %d %f %f\n",xfer.rxBuf[0],bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]),bytestofloat(xfer.rxBuf[7],xfer.rxBuf[8],xfer.rxBuf[9],xfer.rxBuf[10]));
                        
                    }
                    
                    if (xfer.rxCnt > 5 && xfer.rxBuf[2]==9) {
                        printf("MT_DIMMING from %d %d %f %f\n",xfer.rxBuf[0],xfer.rxBuf[11],bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]),bytestofloat(xfer.rxBuf[7],xfer.rxBuf[8],xfer.rxBuf[9],xfer.rxBuf[10]));
                        
                    }
                    if(xfer.rxCnt > 5 && xfer.rxBuf[2]==4){
                        printf("MT_DIMMING from %d\n",xfer.rxBuf[0]);
                        findID((int)xfer.rxBuf[0]);
                        if(it != lista.end()){
                            it->setDimming(bytestofloat(xfer.rxBuf[3],xfer.rxBuf[4],xfer.rxBuf[5],xfer.rxBuf[6]));
                            it->setEnergy();
                        } else {
                            printf("Luminaire %d not found!\n",(int)xfer.rxBuf[0]);
                        }
                        
                    }
                    break;
                case MT_REQUEST_FOR_CALIBRATION:
                    printf("REQUEST_FOR_CALIBRATION from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_CALIBRATION_LED_ON:
                    printf("CALIBRATION_LED_ON from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_CALIBRATION_LED_OFF:
                    printf("CALIBRATION_LED_OFF from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_END_CALIBRATION:
                    printf("END_CALIBRATION from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_REQUEST_JOIN_NETWORK:
                    printf("REQUEST_JOIN_NETWORK from %d\n",xfer.rxBuf[0]);
                    aux = new Desk((int)xfer.rxBuf[0]);
                    lista.push_front(*aux);
                    std::cout << "Id from list: " << lista.front().getID() << '\n';
                    break;
                case MT_REQUEST_JOIN_NETWORK_REPLY_OK:
                    printf("REQUEST_JOIN_NETWORK_REPLY_OK from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_MAX_NODES_IN_NETWORK_REACHED:
                    printf("MAX_NODES_IN_NETWORK_REACHED from %d\n",xfer.rxBuf[0]);
                    break;
                case MT_TODO:
                    printf("TODO from %d\n",xfer.rxBuf[0]);
                    break;   
                default:
                          
						printf("Received %d non identified bytes\n", xfer.rxCnt);
						for (j = 0; j < xfer.rxCnt; j++) {
							printf("%d",xfer.rxBuf[j]);
						}
						printf("\n");
                    break;
            }
            
            
           
            memset(xfer.rxBuf,0,strlen(xfer.rxBuf));
            xfer.rxBuf[0] = '\0';
        }
    }
    printf("Exiting I2C Code!\n");
    i2cClose(handle); /* close master */
    status = close_slave(xfer);
    gpioTerminate();
}


int init_slave(bsc_xfer_t &xfer, int addr) {
    gpioSetMode(18, PI_ALT3);
    gpioSetMode(19, PI_ALT3);
    xfer.control = (addr<<16) | /* Slave address */
    (0x00<<13) | /* invert transmit status flags */
    (0x00<<12) | /* enable host control */
    (0x00<<11) | /* enable test fifo */
    (0x00<<10) | /* invert receive status flags */
    (0x01<<9) | /* enable receive */
    (0x01<<8) | /* enable transmit */
    (0x00<<7) | /* abort and clear FIFOs */
    (0x00<<6) | /* send control reg as 1st I2C byte */
    (0x00<<5) | /* send status regr as 1st I2C byte */
    (0x00<<4) | /* set SPI polarity high */
    (0x00<<3) | /* set SPI phase high */
    (0x01<<2) | /* enable I2C mode */
    (0x00<<1) | /* enable SPI mode */
    0x01 ; /* enable BSC peripheral */
    return bscXfer(&xfer);
}

float bytestofloat(unsigned char a,unsigned char b,unsigned char c,unsigned char d){
	float output;
	//printf("DATA:%d %d %d %d\n",a,b,c,d);
	*((unsigned char*)(&output) + 3) = d;
	*((unsigned char*)(&output) + 2) = c;
	*((unsigned char*)(&output) + 1) = b;
	*((unsigned char*)(&output) + 0) = a;
	
	return output;
}


int close_slave(bsc_xfer_t & xfer) {
    xfer.control = 0;
    return bscXfer(&xfer);
}

//=======================I2C==================================



//=====================GENERAL================================
void keyboard(){
	std::cout<< "Press q to Exit!" << '\n';
	char temp = '\0';
	scanf("%c",&temp);
	//std::cout<< "You pressed " << temp << '\n';
	while(temp != 'q'){
		std::cout<< "Press q to Exit!" << '\n';
		scanf("%c",&temp);
		//std::cout<< "You pressed " << temp << '\n'; 
	}
	std::cout<< "Exiting!" << '\n';
	terminate = 1;
	//i2cClose(handle); /* close master */
    //status = close_slave(xfer);
    //gpioTerminate();
}


int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: server <port>\n"; return 1; }
    std::cout<< "Starting!" << '\n';
    strcpy(port,argv[1]);
    laststart =  time(0);
    std::thread tcpserver(startserver);
    std::thread i2c(starti2c);
    std::thread toexit(keyboard);
    toexit.join();
    tcpserver.join();
    i2c.join();

    std::cout<< "Finished" << '\n';
}

//=====================GENERAL================================
//=======================DATA=================================







//=======================DATA=================================


