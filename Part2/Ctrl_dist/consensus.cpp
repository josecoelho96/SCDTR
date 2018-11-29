#include "consensus.h"
#include <math.h>

// n: nó actual
// d_sum: array com a soma dos dimming level dos outros nós

void consesus(node* n, void* d_sum, int number_nodes){

    // node
    float rho = 0.07; 
    float d[number_nodes];
    float *cost;
    primal_solve(n, rho, number_nodes, &d, cost);
    for ( int j = 0; j < number_nodes; j++) {
        n->d[j] = d[j];
    }

    //Compute average with available data
    for ( int j = 0; j < number_nodes; j++) {
            
            n->d_av[j] = (n->d[j]+d_sum[j])/2;
    }
   
    
    //Update local lagrangians
    for ( int j = 0; j < number_nodes; j++) {

        n->y[j] = n->y[j] + rho*(n->d[j]-n->d_av[j]);
    }
}

int check_feasibility(node* n, void* d, int number_nodes){
    int check;
    float tol = 0.001; //tolerance for rounding errors
    float accum;
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

void init_node(void* d, void* l, void* o, void* L, void* K, void* c, int number_nodes, int i, void* y, void* d_av){
    
    node* n;
    // SOLVE WITH CONSENSUS
    float rho = 0.07;
    // node initialization
    n->index = i;
    n->d = d;
    n->d_av = d_av;
    n->y = y;
    n->k = K;
    double accum = 0;
    for (int j = 0; j < number_nodes; ++j) {
        accum += n->k[j] * n->k[j];
    }
    double norm = sqrt(accum);
    n->n = pow(norm,2);
    n->m = n->n-pow(n->k[0],2);
    n->c = c;
    n->o = o;
    n->L = L;
    
    return n;
}

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

void primal_solve(node* n, float rho, int number_nodes, void* d, float* cost){
    
    float rho = 0.07;
    int j;
    float cost_best = 1000000; //large number
    float sol_unconstrained = 1;
    float sol_boundary_linear = 1;
    float sol_boundary_0 = 1;
    float sol_boundary_100 = 1;
    float sol_linear_0 = 1;
    float sol_linear_100 = 1;
    float z[number_nodes];
    float d_best[number_nodes];
    float d_bl[number_nodes];
    
    for ( int j = 0; j < number_nodes; j++) {
        d_best[j]=-1;
        z[j] = rho*n->d_av[j] - node->y[j];
    }
    
    z[n->index] = z[node->index) - node->c;
                   //unconstrained minimum
                   float d_u[number_nodes];
                   for (int j = 0; j < number_nodes; j++) {
                   d_u[j] = (1/rho)*z[j];
                   }
                   
                   int sol_unconstrained = check_feasibility(n, &d_u, number_nodes);
                   
                   if (sol_unconstrained){
                   
                   //REVISE:
                   float cost_unconstrained = evaluate_cost(n, &d_u, rho, number_nodes);
                   if (cost_unconstrained < cost_best){
                   for (int j = 0; j < number_nodes; j++) {
                   d[j]=d_u[j];
                   }
                   *cost = cost_best;
                   return;
                   //IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
                   //NO NEED TO COMPUTE THE OTHER
                   }
                   
                   }
                   //compute minimum constrained to linear boundary
                   float accum = 0;
                   for (int j = 0; j < number_nodes; j++) {
                   
                   accum += (1/rho)*z[j]*n->k[j];
                   }
                   for (int j = 0; j < number_nodes; j++) {
                   
                   d_bl[j] = (1/rho)*z[j]-n->k[j]/(n->o-n->L+accum);
                   }
                   //check feasibility of minimum constrained to linear boundary
                   int sol_boundary_linear = check_feasibility(n, &d_bl, number_nodes);
                   //compute cost and if best store new optimum
                   if (sol_boundary_linear){
                   float cost_boundary_linear = evaluate_cost(n, &d_bl,rho, number_nodes);
                   if (cost_boundary_linear < cost_best){
                   for (int j = 0; j < number_nodes; j++) {
                   d_best[j]=d_bl[j];
                   }
                   cost_best = cost_boundary_linear;
                   }
                   }
                   
                   //compute minimum constrained to 0 boundary
                   float d_b0[number_nodes];
                   for (int j = 0; j < number_nodes; j++) {
                   d_b0[j] = (1/rho)*z[j];
                   }
                   d_b0[n->index] = 0;
                   
                   //check feasibility of minimum constrained to 0 boundary
                   int sol_boundary_0 = check_feasibility(n, &d_b0, number_nodes);
                   //compute cost and if best store new optimum
                   if (sol_boundary_0){
                   float cost_boundary_0 = evaluate_cost(n, &d_b0,rho, number_nodes);
                   if (cost_boundary_0 < cost_best){
                   for (int j = 0; j < number_nodes; j++) {
                   d_best[j]=d_b0[j];
                   }
                   cost_best = cost_boundary_0;
                   }
                   }
                   
                   //compute minimum constrained to 100 boundary
                   float d_b1[number_nodes];
                   for (int j = 0; j < number_nodes; j++) {
                   d_b1[j] = (1/rho)*z[j];
                   }
                   d_b1[n->index] = 100;
                   //check feasibility of minimum constrained to 100 boundary
                   int sol_boundary_100 = check_feasibility(n, &d_b1, number_nodes);
                   // compute cost and if best store new optimum
                   if (sol_boundary_100) {
                   float cost_boundary_100 = evaluate_cost(n, &d_b1,rho, number_nodes);
                   if (cost_boundary_100 < cost_best){
                   for (int j = 0; j < number_nodes; j++) {
                   d_best[j]=d_b1[j];
                   }
                   cost_best = cost_boundary_100;
                   }
                   }
                   
                   // compute minimum constrained to linear and 0 boundary
                   accum=0;
                   for (int j = 0; j < number_nodes; ++j) {
                   
                   accum += z[j]*n->k[j];
                   }
                   float d_l0[number_nodes];
                   for (int j = 0; j < number_nodes; j++) {
                   d_l0[j] = (1/rho)*z[j]-(1/n->m)*n->k[j]*(n->o-n->L)+((1/rho)/n->m)*n->k[j]*(n->k[n->index]*z[n->index]-accum);
                   }
                   d_l0[n->index] = 0;
                   //check feasibility of minimum constrained to linear and 0 boundary
                   int sol_linear_0 = check_feasibility(n, &d_l0, number_nodes);
                   // compute cost and if best store new optimum
                   if (sol_linear_0){
                   float cost_linear_0 = evaluate_cost(n, &d_l0,rho, number_nodes);
                   if (cost_linear_0 < cost_best){
                   for (int j = 0; j < number_nodes; j++) {
                   d_best[j]=d_l0[j];
                   }
                   cost_best = cost_linear_0;
                   }
                   }
                   
                   //compute minimum constrained to linear and 100 boundary
                   float d_l1[number_nodes];
                   for (int j = 0; j < number_nodes; j++) {
                   d_l1[j] = (1/rho)*z[j]-(1/n->m)*n->k[j]*(n->o-n->L+100*n->k[n->index])+((1/rho)/n->m)*n->k[j]*(n->k[n->index]*z[n->index]-accum);
                   }
                   d_l1[n->index] = 100;
                   //check feasibility of minimum constrained to linear and 0 boundary
                   int sol_linear_0 = check_feasibility(n, &d_l1, number_nodes);
                   // compute cost and if best store new optimum
                   if (sol_linear_0){
                   float cost_linear_0 = evaluate_cost(n, &d_l1,rho, number_nodes);
                   if (cost_linear_0 < cost_best){
                   for (int j = 0; j < number_nodes; j++) {
                   d_best[j]=d_l1[j];
                   }
                   cost_best = cost_linear_0;
                   }
                   }
                   for (int j = 0; j < number_nodes; j++) {
                   d[j]=d_best[j];
                   }
                   *cost = cost_best;
                   return;
                   
                   
                   }
