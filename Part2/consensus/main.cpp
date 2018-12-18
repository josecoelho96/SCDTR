//
//  main.cpp
//  
//
//  Created by David Teles on 29/11/2018.
//

#include <stdio.h>
#include "consensus.h"
#include <stdio.h>
#include <stdlib.h>



int main(){

    // José
    // A matriz K é feita aqui
    
    
    
    //float K[4] = {2, 1 , 1 , 2};
    //float K1[2] = {2,1};
    //float K2[2] = {1,2};

    // Aqui definimos os valores de tecto, ou seja, os L's, acho que pode ser à pata para cada um
    
    //float L[2] = {80,150};
    

    // Fazer uma leitura da iluminância actual e dar esse valor aos o's

    //float o1 = 50;
    //float o2 = 50;


    float c[2] = {1,3}; 
     
    float o[2] = {50,50};
    float rho = 0.07;
    
    //node initialization

    float d1[2] = {0,0};
    float l1[2] = {0,0};
    float d_av1[2] = {0,0};
    float y1[2] = {0,0};
    struct node* n1 = init_node(d1, l1, o[0], L[0], K1, c[0], 2, 0, y1, d_av1);
    printf("n1=%f   ,     n1=%f\n", n1->o, n1->L);



    //Esta parte não interessa
    /*
    //node 2 initialization
    float d2[2] = {0,0};
    float l2[2] = {0,0};
    float d_av2[2] = {0,0};
    float y2[2] = {0,0};
    struct node* n2 = init_node(d2, l2, o[1], L[1], K2, c[1], 2, 1,y2, d_av2);
    printf("n2=%f   ,     n2=%f\n", n2->o, n2->L);
    */


// Aqui fica o loop do Arduino onde está sempre a fazer as instruções seguintes



    // Aqui é a parte de iterações, não sei se preferem fazer com interrupts ou assim
    // Acho que se iam fazendo de 10 a 10 iterações, ou seja, fazem 10 iterações entre eles, sabe o valor que cada LED deve ter e depois usa a parte 1 do projecto para o LED chegar
    // ao valor pretendido. Depois lê a luminosidade externa e volta a fazer o mesmo a partir daqui
    //iterations
    for (int i=2; i<=10; i++){

        // here we put a for to look at all nodes and do the primal solve
        // node actual
        // Faz consensus para o nó onde está o código
        consesus(n1, 2);
        
        /* Não interessa
        //node2
        consesus(n2, 2);
        */


       // Depois de fazer o consensus precisa de enviar para todos os outros nós o dimming level, d

       // Função enviar dimming level

       // Depois tem de receber os dimming level de todos os nós depois de eles também terem realizado o consensus

       // Função receber os dimming level de todos

        // Compute average with available data
        float d_sum[2];
        for (int j=0; j<2; j++){
            d_sum[j]=n1->d[j]+n2->d[j];
        }


        //node 1
        final_values(n1, d_sum, 2, rho);

        /* Não interessa
        //node 2
        final_values(n2, d_sum, 2, rho);
        */
    }

    // No fim das iterações sabe que valor há de dar aos LEDS (usar o feedforward e feedback)
    float* d_final = n1->d_av;
    printf("d1=%f   ,     d2=%f\n", n1->d_av[0], n1->d_av[1]);
    float l_final[2];
    float accum;
    for (int i=0; i<2; i++){
        accum = 0;
        for(int j=0; j<2; j++){
            if (i == 0){
                accum += n1->k[j]*d_final[j];
            }
            //if (i == 1){
              //  accum += n2->k[j]*d_final[j];
            //}
        }
        l_final[i] = accum;

    }
    l_final[0]=l_final[0]+n1->o;
    //l_final[1]=l_final[1]+n2->o;

    printf("l1=%f   ,     l2=%f\n", l_final[0], l_final[1]);


    // Calcula novas iluminâncias externas

    // d*K dá a previsão da iluminância sem acção externa
    // Medir com a parte 1 do projecto a iluminância para aquele nó e subtrair à previsão

    // Fim do loop de todo o Arduino, volta a realizar as intruções anteriores

    free(n1);
    //free(n2);
    return 0;
}