#include <iostream>
#include <stdio.h>     
#include <stdlib.h>    

#include <list>
#include "Desk.h"
std::list<Desk> lista;



Desk findID(int id){
    
    std::list<Desk>::iterator it;
    std::cout << "Searching for ID: " << id <<'\n';
    for (it = lista.begin(); it != lista.end(); ++it) {
        std::cout << "ID: " << it->getID() << '\n';
        if (it->getID() == id) {
            break;
        }
    }
    
    return *it;
}

int main() {
    Desk *aux;
    for (int i = 1; i<= 10; i++) {
        aux = new Desk(i);
        lista.push_front(*aux);
    }

    
    std::cout << "ID found: " << findID(2).getID() << '\n';
    return 0;
    
}
