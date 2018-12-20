#ifndef DESK_H
#define DESK_H

#include <list>
#include <time.h>
#include<sys/time.h>


class Tupple{
private:
    long time;
    float valuetosave;
public:
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

    float aux;
    aux = 0;
    /*
    if (this->time.size() > 1){
        for ( int i=1; i< this->time.size(); i++){
            aux = aux + this->dimming.get(i-1)*(this->time.get(i)-this->time.get(i-1));
        }
    }
    */
    this->Energy = aux;
}

// set confort error
void setConfortError(){
    
    float aux1, aux2, aux3;
    aux1 = 0;
    aux2 = 0;
/*
    for ( int i=0; i<this->time.size(); i++){
        if ((this->il_ControlRef.get(i)-this->iluminance.get(i)) > 0){
            aux1 = this->il_ControlRef.get(i);
        }
        else{
            aux1 = 0;
        }

        aux2 = aux2 + aux1;
    }

    aux2 = aux2/this->time.size();*/

    this->ConfortError = aux2;
}

// Set confort flicker
void setConfortFlicker(){
    
    float aux;
    aux = 0;
    float f;
    f =0;
    /*
    if (this->time.size() > 2){
        for ( int i=2; i<this->time.size(); i++){
            if ((this->iluminance.get(i)-this->iluminance.get(i-1))*(this->iluminance.get(i-1)-this->iluminance.get(i-2)) < 0){
                f = abs(long(this->iluminance.get(i)-this->iluminance.get(i-1)))+abs(long(this->iluminance.get(i-1)-this->iluminance.get(i-2)))/(2*0.000031875);
            }    
            aux = aux + f;
        }
    }

    aux = aux/this->time.size();*/

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
    struct timeval tv;
    gettimeofday(&tv,NULL);
    this->time = (((long)tv.tv_sec)*1000)+(tv.tv_usec/1000);


    for (this->it = this->iluminance->begin(); this->it != this->iluminance->end() || this->it->getTime<(((long)tv.tv_sec+60)*1000)+(tv.tv_usec/1000); ; ++this->it) {
        temp[counter]=this->it->getValue();
        counter++;
        temp[counter]=';';
        counter++;
        temp[counter]=this->it->getTime();
        counter++;
        temp[counter]=';';
        counter++;
    }
        temp[counter-1]='\n';
    
}
    
void getLastDimming(char* temp){
    int counter = 0;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    this->time = (((long)tv.tv_sec)*1000)+(tv.tv_usec/1000);
    
    for (this->it = this->dimming->begin(); this->it != this->dimming->end() || this->it->getTime<(((long)tv.tv_sec+60)*1000)+(tv.tv_usec/1000); ; ++this->it) {
        temp[counter]=this->it->getValue();
        counter++;
        temp[counter]=';';
        counter++;
        temp[counter]=this->it->getTime();
        counter++;
        temp[counter]=';';
        counter++;
    }
    temp[counter-1]='\n';
    
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
