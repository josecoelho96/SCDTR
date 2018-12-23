#include <iostream>
#include <stdio.h>     
#include <stdlib.h>
#include <unistd.h>

#include <list>
#include "Desk.h"
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

int main() {
    char msg[100];
    Desk *aux;
    for (int i = 1; i<= 10; i++) {
        aux = new Desk(i);
        
        lista.push_front(*aux);
    }
    for (int i = 1; i<= 10; i++) {
        findID(i);
        it->setIluminance(1.1);
        usleep(100);
        it->setIluminance(1.2);
        usleep(200);
        it->setIluminance(1.3);
        usleep(300);
        it->setIluminance(1.4);
    }
    
    findID(2);
    it->getLastLuminance(msg);
    std::cout << "ID found: " << it->getID() << " value " << it->getIluminance().getValue() << " time " << it->getIluminance().getTime() <<'\n';
    std::cout << msg << '\n' << std::endl;
    std::cout << "Bye\n";
    return 0;
    
}
