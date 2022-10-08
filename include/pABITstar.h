#ifndef PLANNING_H
#define PLANNING_H
#include "planner.h"


//=================== planning ====================================
class pABITstar : public Planner
{
public:
    //------------- funzioni ------------------
    //base
    pABITstar(GraphWidget* widget, PlanningData* PD, Solution* S); //costruttore
    ~pABITstar() override; //devo per forza overridare il distruttore
    void start() override; //inizio planning

    //ricerca
    void expandVertex(Node* v);
    float bestVertexQueueValue();
    float bestEdgeQueueValue();
    Node* bestInVertexQueue();
    std::pair<Node*, Node*> bestInEdgeQueue();
    void radius(int q) override;

    //euristiche
    float gTf(Node* x);
    float f_estimated(Node* x);
    float g_estimated(Node* x);
    float h_estimated(Node* x);

    //sampling
    int prune();
    std::set<Node*> sample(int m, float Cbest, float Cmin, Eigen::Vector3f x_center, Eigen::Matrix3f C); //overload
    std::set<Node*> sampleFreeSpace(int m); //overload
    std::set<Node*> sampleEllipsoid(int m, float Cbest, float Cmin, Eigen::Vector3f x_center, Eigen::Matrix3f C); //overload
    Eigen::Vector3f sampleUnitBall(); //overload


    //ausiliari
    float cost(std::pair<Node*, Node*> e); //costo edge
    void updateCbest() override; //aggiorno soluzione comune

protected:
    //------ variabili -------------------------
    std::set<Node*> Xunconn;
    std::map<Node*, float> gT;

    //-------- funzioni ------------------------
    bool terminationCondition(float Cmin, std::chrono::time_point<std::chrono::high_resolution_clock> t_start) override; //check termination condition
    void stop(std::chrono::time_point<std::chrono::high_resolution_clock> t_start) override; //stop planning (per tutti i thread)
};

#endif // PLANNING_H
