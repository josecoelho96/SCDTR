struct node {
    int index;
    void* d;
    void* d_av;
    void* y;
    void* k;
    float n;
    float m;
    float c;
    float o;
    float L;
    };


void init_node(void*, void*, void*, void*, void*, void*, int, int);
int check_feasibility(struct*, void*, int);
float evaluate_cost(node*, void*, float, int);
void primal_solve(node*, float, int, void*, float*);