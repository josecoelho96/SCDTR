#include <consensus.h>
#include <math.h> 

int check_feasibility(node* n, void* d, int number_nodes){

    float tol = 0.001; //tolerance for rounding errors
    if (d[n->index] < 0-tol){
    check = 0; 
    return check;  
    } 
    if (d[n->index] > 100+tol){

    check = 0; 
    return check;  
    } 
    for (int j = 0; j < number_nodes; ++j) {
        accum += d[j]*d[j];
    }

    if (accum < n->L-n->o-tol){ 
        check = 0; 
        return check; 
    }
    check = 1;
    return check;

}
