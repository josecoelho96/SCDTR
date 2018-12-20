#ifndef DESK_H
#define DESK_H

#include <list>



class Desk{
private:
    std::list<float> time;
	std::list<float> iluminance;
    std::list<float> dimming;
    float duty_cicle;
    bool OccupancyState;
    float il_LowerBound;
    float il_External;
    std::list<float> il_ControlRef;
    float Energy;
    float ConfortError;
    float ConfortFlicker;
    int id;

public:/*
	Desk(int aux);
    void setDutyCicle(float duty);
    void setOccupancyState(float state);
    void setil_LowerBound(float L);
    void setil_External(float o);
    void setEnergy();
    void setConfortError();
    void setConfortFlicker();
    void setTime(float t);
    void setIluminance(float l);
    void setDimming(float d);
    void setControlRef(float ref);
    
    float getDutyCicle();
    bool getOccupancyState();
    float getil_LowerBound();
    float getil_External();
    float getEnergy();
    float getConfortError();
    float getConfortFlicker();
    float getTime();
    float getIluminance();
    float getDimming();
    float getControlRef();
    int getID();*/
    


//Constructor
Desk(int aux) {
	
	this->id = aux;
    this->duty_cicle = 0;
    this->OccupancyState = 0;
    this->il_LowerBound = 0;
    this->il_External = 0;
    this->Energy = 0;
    this->ConfortError = 0;
    this->ConfortFlicker = 0;
    //this->il_ControlRef = new std::list();
    this->il_ControlRef.push_back(0);
    //this->time = new std::list();
    this->time.push_back(0);
    //this->iluminance = new std::list();
    this->iluminance.push_back(0);
    //this->dimming =  new std::list();
    this->dimming.push_back(0);
    
}

// set dutycicle
void setDutyCicle(float duty){
    
    this->duty_cicle = duty;
}

// set occupancy state
void setOccupancyState( float state){

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
*/
    aux2 = aux2/this->time.size();

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
*/
    aux = aux/this->time.size();

    this->ConfortFlicker = aux;
}

// set time
void setTime(float t){
    
    this->time.push_back(t);
}

// set Iluminance
void setIluminance(float l){
    
    this->iluminance.push_back(l);
}

// set Dimming
void setDimming(float d){
    
    this->dimming.push_back(d);
}

// set iluminance control reference
void setControlRef(float ref){
    
    this->il_ControlRef.push_back(ref);
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

// get time
float getTime(){

    int n = this->time.size();
    return this->time.back();
}

// get iluminance
float getIluminance(){

    int n = this->iluminance.size();
    return this->iluminance.back();
}

// get ID
int getID(){
    return this->id;
}


// get iluminance
float getDimming(){

    int n = this->dimming.size();
    return this->dimming.back();
}

// get iluminance control ref
float getControlRef(){
    int n = this->il_ControlRef.size();
    return this->il_ControlRef.back();

}
};
#endif
