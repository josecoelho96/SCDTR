#include <consensus.h>
#include <math.h> 

void init_node(float* d, float* l, void* o, void* L, void* K, void* c, int number_nodes, int i){

node* n;
// SOLVE WITH CONSENSUS
float rho = 0.07;
// node initialization
n.index = i;
n.d = d;
n.d_av = d_av;
n.y = y;
n.k = K; 
double accum = 0;
for (int j = 0; j < number_nodes; ++j) {
    accum += n.k[j] * n.k[j];
}
double norm = sqrt(accum);
n.n = pow(norm,2);
n.m = n.n-pow(n.k[0],2);
n.c = c;
n.o = o;
n.L = L;

return n;
}
