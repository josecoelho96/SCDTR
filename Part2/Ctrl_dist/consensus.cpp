#include <consensus.h>
#include <math.h>

// n: nó actual
// d_sum: array com a soma dos dimming level dos outros nós

void consesus(node* n, void* d_sum, int number_nodes){

    // node
    float rho = 0.07; 
    float d[number_nodes];
    float *cost;
    primal_solve(n, rho, number_nodes, &d, cost);
    for ( j = 0; j < number_nodes; j++) {
        n->d[j] = d[j];
    }

    //Compute average with available data
    for ( int j = 0; j < number_nodes; j++) {
            
            n->d_av[j] = (n->d[j]+d_sum[j])/2;
    }
    n->d_av = (node1.d+node2.d)/2;
    
    //Update local lagrangians
    for ( j = 0; j < number_nodes; j++) {

        n->y[j] = n->y[j] + rho*(n->d[j]-n->d_av[j]);
    }
}