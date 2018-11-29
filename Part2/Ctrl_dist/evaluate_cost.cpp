#include <consensus.h>
#include <math.h> 

float evaluate_cost(node* n, void* d, float rho, int number_nodes){

    float cost;
    float accum1, accum2;
    for (int j = 0; j < number_nodes; ++j) {
         
        accum1 += n->y[j]*(d[j] - n->d_av[j]);
        accum2 += d[j] - n->d_av[j];
    }
    double norm = sqrt(accum2);

    cost =  n->c*d[n->index] + accum1 + rho/2*pow(accum2,2);
    return cost;
}
