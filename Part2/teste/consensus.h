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


struct node* init_node(float* , float* , float , float , float* , float , int , int , float* , float*);
float check_feasibility(struct node*, float*, int);
float evaluate_cost(struct node*, float*, float, int);
void primal_solve(struct node*, float, int, float*);
void final_values(struct node* , float*, int, float);
void consesus(struct node* , int );