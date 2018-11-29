struct node {
    int index;
    float* d;
    float* d_av;
    float* y;
    float* k;
    float n;
    float m;
    float c;
    float o;
    float L;
    };


node* init_node(float*, float*, float*, float*, float*, float*, int, int);
float check_feasibility(node*, float*, int);
float evaluate_cost(node*, float*, float, int);
void primal_solve(node*, float, int, float*, float*);
