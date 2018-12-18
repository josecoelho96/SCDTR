#include "consensus.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// n: nó actual
// d_sum: array com a soma dos dimming level dos outros nós

void consesus(struct node* n, int number_nodes){

    // node
    float rho = 0.07; 
    float d[number_nodes];
    
    primal_solve(n, rho, number_nodes, d);
    for ( int j = 0; j < number_nodes; j++) {
        n->d[j] = d[j];
    }
    
}

void final_values(struct node* n, float* d_sum, int number_nodes, float rho){

    //Compute average with available data
    for ( int j = 0; j < number_nodes; j++) {
            
            n->d_av[j] = (d_sum[j])/2;
    }
   
    
    //Update local lagrangians
    for ( int j = 0; j < number_nodes; j++) {

        n->y[j] = n->y[j] + rho*(n->d[j]-n->d_av[j]);
    }
}




float check_feasibility(struct node* n, float* d, int number_nodes){
    float check;
    float tol = 0.001; //tolerance for rounding errors
    float accum =0;
    if (d[n->index] < 0-tol){
        check = 0;
        return check;
    }
    if (d[n->index] > 100+tol){
        
        check = 0;
        return check;
    }
    for (int j = 0; j < number_nodes; ++j) {
        accum += d[j]*n->k[j];
    }
    
    if (accum < n->L-n->o-tol){
        check = 0;
        return check;
    }
    check = 1;
    return check;
    
}

struct node* init_node(float* d, float* l, float o, float L, float* K, float c, int number_nodes, int i, float* y, float* d_av){
    
    struct node* n = (struct node*) malloc(sizeof(struct node));

    // node initialization
    n->index = i;
    n->d = d;
    n->d_av = d_av;
    n->y = y;
    n->k = K;
    float k_node = n->k[n->index];
    double accum = 0;
    for (int j = 0; j < number_nodes; ++j) {
        accum += n->k[j] * n->k[j];
    }
    double norm = sqrt(accum);
    n->n = pow(norm,2);
    n->m = n->n-pow(k_node,2);
    n->c = c;
    n->o = o;
    n->L = L;
    
    return n;
}

float evaluate_cost(struct node* n, float* d, float rho, int number_nodes){
    
    float cost;
    float accum1, accum2;
    accum1 = 0;
    accum2 = 0;
    for (int j = 0; j < number_nodes; ++j) {
        
        accum1 += n->y[j]*(d[j] - n->d_av[j]);
        accum2 += pow((d[j] - n->d_av[j]),2);
    }
    double norm = sqrt(accum2);
    
    cost =  n->c*d[n->index] + accum1 + (rho/2)*pow(norm,2);
    return cost;
}

void primal_solve(struct node* n, float rho, int number_nodes, float* d){
    
    float cost;
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
        z[j] = rho*n->d_av[j] - n->y[j];
    }
    
    z[n->index] = z[n->index] - n->c;
    //unconstrained minimum
    float d_u[number_nodes];
    for (int j = 0; j < number_nodes; j++) {
        d_u[j] = (1/rho)*z[j];
    }
    
    sol_unconstrained = check_feasibility(n, d_u, number_nodes);
    
    if (sol_unconstrained){
    
        //REVISE:
        float cost_unconstrained = evaluate_cost(n, d_u, rho, number_nodes);
        if (cost_unconstrained < cost_best){
            for (int j = 0; j < number_nodes; j++) {
                d[j]=d_u[j];
            }
            cost = cost_unconstrained;
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
    
        d_bl[j] = (1/rho)*z[j]-n->k[j]/n->n*(n->o-n->L+accum);
    }
    //check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(n, d_bl, number_nodes);
    //compute cost and if best store new optimum
    if (sol_boundary_linear){
        float cost_boundary_linear = evaluate_cost(n, d_bl,rho, number_nodes);
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
    sol_boundary_0 = check_feasibility(n, d_b0, number_nodes);
    //compute cost and if best store new optimum
    if (sol_boundary_0){
        float cost_boundary_0 = evaluate_cost(n, d_b0,rho, number_nodes);
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
    sol_boundary_100 = check_feasibility(n, d_b1, number_nodes);
    // compute cost and if best store new optimum
    if (sol_boundary_100) {
        float cost_boundary_100 = evaluate_cost(n, d_b1,rho, number_nodes);
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
        d_l0[j] = (1/rho)*z[j]-(1/n->m)*n->k[j]*(n->o-n->L)+(1/rho/n->m)*n->k[j]*(n->k[n->index]*z[n->index]-accum);
    }
    d_l0[n->index] = 0;
    //check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(n, d_l0, number_nodes);
    // compute cost and if best store new optimum
    if (sol_linear_0){
        float cost_linear_0 = evaluate_cost(n, d_l0,rho, number_nodes);
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
        d_l1[j] = (1/rho)*z[j]-(1/n->m)*n->k[j]*(n->o-n->L+100*n->k[n->index])+(1/rho/n->m)*n->k[j]*(n->k[n->index]*z[n->index]-accum);
    }
    d_l1[n->index] = 100;
    //check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(n, d_l1, number_nodes);
    // compute cost and if best store new optimum
    if (sol_linear_0){
        float cost_linear_0 = evaluate_cost(n, d_l1,rho, number_nodes);
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
    cost = cost_best;
    return;
    
    
    }