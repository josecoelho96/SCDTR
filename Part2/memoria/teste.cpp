#include <iostream>
#include <stdio.h>     
#include <stdlib.h>    

#include "Desk.h"
#include "List.h"

int main() {

  List<Desk> list;
  Desk d;
  d.setTime(1);
  d.setIluminance(4);
  d.setEnergy();
  d.setil_LowerBound(100);
  // dá merda aqui
  list.push_back(d);
  printf("fuckkk\n");
  printf("R:%f\n", list.get(0).getTime());
  printf("R:%f\n", list.get(0).getEnergy());
  printf("R:%f\n", list.get(0).getIluminance());
  return 0;

}