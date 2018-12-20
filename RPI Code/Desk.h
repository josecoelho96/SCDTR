#ifndef DESK_H
#define DESK_H

#include <list>
#include <time.h>
#include<sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


class Tupple{
private:
    long time;
    float valuetosave;
public:
    
    Tupple(){
       
    }
    Tupple(float receivedvalue){

        this->setValue(receivedvalue);
        this->setTime();
    }
    
    void setValue(float receivedvalue){
        this->valuetosave = receivedvalue;
    }
    
    void setTime(){
        struct timeval tv;
        gettimeofday(&tv,NULL);
        this->time = (((long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
    }
    
    float getValue(){
        return this->valuetosave;
    }
    
    long getTime(){
        return this->time;
    }
};
class Desk{
private:
    std::list<Tupple>::iterator it;
	std::list<Tupple> iluminance;
    std::list<Tupple> dimming;
    std::list<Tupple> il_ControlRef;
    float duty_cicle;
    bool OccupancyState;
    float il_LowerBound;
    float il_External;

    float Energy;
    float ConfortError;
    float ConfortFlicker;
    int id;

public:

//Constructor
Desk(int aux) {
    Tupple *auxt;
	this->id = aux;
    this->duty_cicle = 0;
    this->OccupancyState = 0;
    this->il_LowerBound = 0;
    this->il_External = 0;
    this->Energy = 0;
    this->ConfortError = 0;
    this->ConfortFlicker = 0;
    
    auxt = new Tupple(0.0);
    this->il_ControlRef.push_front(*auxt);
    
    auxt = new Tupple(0.0);
    this->iluminance.push_front(*auxt);
    auxt = new Tupple(0.0);
    this->dimming.push_front(*auxt);
    
}

// set dutycicle
void setDutyCicle(float duty){
    
    this->duty_cicle = duty;
}

// set occupancy state
void setOccupancyState( bool state){

    this->OccupancyState = state;
}

// set iluminance lower bound
void setil_LowerBound(float L){
    
    this->il_LowerBound = L;
}

// set external iluminance
void setil_External( float o){

    this->il_External = o;
}

// set energy
void setEnergy(){
    Tupple *auxt;
    float aux;
    bool first = true;
    aux = 0;
    
    if (this->dimming.size() > 1){
        for (this->it = this->dimming.begin(); this->it != this->dimming.end(); ++this->it){
            if (first == false) {
                aux = aux + auxt->getValue()*(this->it->getTime()-auxt->getTime());
            }
            *auxt = *(this->it);
            
        }
    }
    
    this->Energy = aux;
}

// set confort error
void setConfortError(){
    std::list<Tupple>::iterator it2;
    float aux1, aux2, aux3;
    long time_int, time_fin;
    aux1 = 0;
    aux2 = 0;
    it2 = this->iluminance.begin();
    ++it2;
    time_int = it2->getTime();
     for (this->it = this->il_ControlRef.begin(); this->it != this->il_ControlRef.end(); ++this->it){
         
            if ((this->it->getValue()-(it2->getValue())) > 0){
                aux1 = this->it->getValue();
            }
            else{
                aux1 = 0;
            }
         if (it2 == this->iluminance.end()) {
             time_fin = it2->getTime();
             ++it2;
         } else {
             break;
         }
         
            aux2 = aux2 + aux1;
        }
    
    aux2 = aux2/(time_fin-time_int);

    this->ConfortError = aux2;
}

// Set confort flicker
void setConfortFlicker(){
    Tupple *auxt1, *auxt2;
    bool first = true, second = false;
    float aux = 0;
    long time_int, time_fin;
    float f = 0, f1 = 0, f2 = 0;

    
    
    if (this->iluminance.size() > 2){
        
        for (this->it = this->iluminance.begin(); this->it != this->iluminance.end(); ++this->it){
            if (first) {
                time_int=this->it->getTime();
                *auxt1 = *(this->it);
            } else if (second){
                *auxt2 = *auxt1;
                *auxt1 = *(this->it);
            } else {
                if ((this->it->getValue()-(auxt1)->getValue())*((auxt1)->getValue()-(auxt2)->getValue()) < 0){
                 
                    f1 = this->it->getValue()-(auxt1)->getValue();
                    f2 = auxt1->getValue() - auxt2->getValue();
                    
                    f = (abs((long)f1)+abs((long)f2))/(2*0.000031875);
                 }
                
                time_fin=this->it->getTime();
                *auxt2 = *auxt1;
                *auxt1 = *(this->it);
            }
            
            aux = aux + f;
        }
    }

    aux = aux/(time_fin-time_int);

    this->ConfortFlicker = aux;
}


// set Iluminance
void setIluminance(float l){
    Tupple *aux2;
    aux2 = new Tupple(l);
    this->iluminance.push_front(*aux2);
    //std::cout << "value received: " << l << " value saved: " << this->iluminance.front().getValue() << '\n';
}

// set Dimming
void setDimming(float d){
    Tupple *aux;
    aux = new Tupple(d);
    this->dimming.push_front(*aux);
}

// set iluminance control reference
void setControlRef(float ref){
    Tupple *aux;
    aux = new Tupple(ref);
    this->il_ControlRef.push_front(*aux);
}

// get duty cicle
float getDutyCicle(){
    return this->duty_cicle;
}

// get occupancy state
bool getOccupancyState(){
    return this->OccupancyState;
}

// get ilumminance lower bound
float getil_LowerBound(){
    return this->il_LowerBound;
}

// get external iluminance
float getil_External(){
    return this->il_External;
}

// get energy
float getEnergy(){
    return this->Energy;
}

// get confort error
float getConfortError(){
    return this->ConfortError;
}

// get Confort flicker
float getConfortFlicker(){
    return this->ConfortFlicker;
}

void getLastLuminance(char* temp){
    int counter = 0;
    char auxstr[25];
    struct timeval tv;
    gettimeofday(&tv,NULL);


    for (this->it = this->iluminance.begin(); this->it != this->iluminance.end() && this->it->getTime()<(((long)tv.tv_sec+60)*1000)+(tv.tv_usec/1000); ++this->it) {
        sprintf(auxstr,"%f;%ld;",this->it->getValue(),this->it->getTime());
        strcat(temp,auxstr);
        counter+=strlen(auxstr);
    }
    sprintf(auxstr,"\n");
    strcat(temp,auxstr);
    temp[counter+1]='\n';
    printf("%d=%c",counter,temp[counter+1]);
    temp[counter+2]='\0';
}
    
void getLastDimming(char* temp){
    int counter = 0;
    char auxstr[25];
    struct timeval tv;
    gettimeofday(&tv,NULL);
    
    
    for (this->it = this->dimming.begin(); this->it != this->dimming.end() && this->it->getTime()<(((long)tv.tv_sec+60)*1000)+(tv.tv_usec/1000); ++this->it) {
        sprintf(auxstr,"%f;%ld;",this->it->getValue(),this->it->getTime());
        strcat(temp,auxstr);
        counter+=strlen(auxstr);
    }
    sprintf(auxstr,"\n");
    strcat(temp,auxstr);
    temp[counter+1]='\n';
    printf("%d=%c",counter,temp[counter+1]);
    temp[counter+2]='\0';
    
    
}
    
    
void reset(){
    this->Energy = 0;
    this->ConfortError = 0;
    this->ConfortFlicker = 0;
}

// get iluminance
Tupple getIluminance(){

    return this->iluminance.front();
}

// get ID
int getID(){
    return this->id;
}


// get iluminance
Tupple getDimming(){

    return this->dimming.front();
}

// get iluminance control ref
Tupple getControlRef(){
    return this->il_ControlRef.front();

}
    
Tupple findID(std::list<Tupple> *list){
        
        
    //std::cout << "Searching for ID: " << id <<'\n';
    for (this->it = list->begin(); this->it != list->end(); ++this->it) {
     
    }
        
    return *this->it;
}
};
#endif
