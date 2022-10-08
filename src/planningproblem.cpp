#include "planningproblem.h"

//costruttore: creo soluzione e Plannings
PlanningProblem::PlanningProblem(std::vector<GraphWidget*> widgets, std::vector<PlanningData*> PDs1, std::vector<PlanningData*> PDs2)
{
    this->S = new Solution(); //creo subito Soluzione condivisa
    auto t1 = PDs1.size();
    auto t2 = PDs2.size();

    //creo istanze planning ABIT
    for (int i=0; i<t1; i++){
        this->Ps1.push_back(new pABITstar(std::ref(widgets[i]), PDs1[i], std::ref(this->S)));
    }
    //creo istanze planning RRT
    for (int i=0; i<t2; i++){
        this->Ps2.push_back(new RRTstar(std::ref(widgets[i+t1]), PDs2[i], std::ref(this->S)));
    }
}


//distruttore (usato pointers --> elimino a mano)
PlanningProblem::~PlanningProblem(){
    //delete Planning objects
    for (auto i:this->Ps1){
        delete i;
    }
    for (auto i:this->Ps2){
        delete i;
    }
    //delete Soluzion objects
    delete this->S;
}


//lancio i singoli plan su chiamata
Solution* PlanningProblem::getPath(){
    //lancio ABIT
    for (auto i:this->Ps1){
        this->threads.push_back(std::thread(&pABITstar::start,i));
    }
    //lancio RRT
    for (auto i:this->Ps2){
        this->threads.push_back(std::thread(&RRTstar::start,i));
    }
    //aspetto fine esecuzione
    for (std::thread &t : this->threads){
        if (t.joinable())
            t.join();
    }
    return this->S;
}


//cambio dinamicamente livello ottimalita
void PlanningProblem::setOptThreshold(float threshold){
    for(auto p : this->Ps1){
        p->threshold = threshold;
    }
    for(auto p : this->Ps2){
        p->threshold = threshold;
    }

}
