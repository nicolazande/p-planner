#ifndef RRTSTAR_H
#define RRTSTAR_H
#include "components.h"
#include "planner.h"


class RRTstar : public Planner
{
public:

    //------------ funzioni -------------------
    RRTstar(GraphWidget* widget, PlanningData* PD, Solution* S); //costruttore
    ~RRTstar() override; //distruttore
    void start() override; //lancio planning

    //sampling
    Node* sample();

    //costo
    float cost(Node* q);

    //ricerca
    void searchGoalParent();
    void rewire(Node* q);
    void choseParent(Node* q);
    void findNeighbours(Node* q_new);
    Node* nearestNeighbour(Node* x);

    //generale
    Node* steer(Node* q_near, Node* q_rand);
    bool isFound(Node* q);
    bool terminationCondition(float Cmin, std::chrono::time_point<std::chrono::high_resolution_clock> t_start) override;
    void stop(std::chrono::time_point<std::chrono::high_resolution_clock> t_start) override;
};

#endif // RRTSTAR_H
