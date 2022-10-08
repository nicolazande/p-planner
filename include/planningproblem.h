#ifndef PLANNINGPROBLEM_H
#define PLANNINGPROBLEM_H
#include <vector>
#include <thread>
//includo oggetti per associazioni
#include "solution.h"
#include "planner.h"
#include "pABITstar.h"
#include "rrtstar.h"
#include "env.h"
//forward declarations
class PlanningData;

class PlanningProblem
{
public:
    //------ variabili ------------------------------------
    std::vector<std::thread> threads; //vettore di threads
    Solution* S; //Soluzione condivisa (by reference a Plannings)
    std::vector<pABITstar*> Ps1; //vettore di ABIT
    std::vector<RRTstar*> Ps2; //vettore di ABIT

    //------ funzioni -------------------------------------
    PlanningProblem(std::vector<GraphWidget*> widgets, std::vector<PlanningData*> PDs1, std::vector<PlanningData*> PDs2); //costruttore
    ~PlanningProblem(); //distruttore
    Solution* getPath(); //chiamata inizio planning
    void setOptThreshold(float threshold); //cambio valore ottimalita (per visualizzazione)
};

#endif // PLANNINGPROBLEM_H
