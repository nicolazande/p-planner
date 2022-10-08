#ifndef SOLUTION_H
#define SOLUTION_H
#include <iostream>
#include <vector>

//costanti
const float INF = 1e6;

class Solution
{
public:
    //------ variabili -----------------------------------
    float Cbest;
    bool state; //stato (trovato/non trovato)
    float time = INF;
    std::string winner = "";

    //addizionali per benchmark
    std::vector<std::pair<int, std::pair<float, float>>> results_vec; //output tutti thread results_vec = {threadID, {time, cost}}


    //------ funzioni ------------------------------------
    Solution(); //costruttore
    void updateTree(int T); //aggiorno Tree
};

#endif // SOLUTION_H
