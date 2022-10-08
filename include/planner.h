#ifndef PLANNER_H
#define PLANNER_H
#include <unistd.h>
#include <stdlib.h>
#include <set>
#include <map>
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/Geometry>
#include <chrono>
//includo header per associazioni
#include "planningdata.h"
#include "solution.h"
#include "env.h"
#include "graphwidget.h"
#include "components.h"



//=================== planning ====================================
class Planner
{
public:
    //-------- variabili -----------------------
    Solution* S;
    PlanningData* PD;
    float threshold; //rapporto = C / Cmin

    //------------- funzioni ------------------
    //base
    Planner(GraphWidget* widget, PlanningData* PD, Solution* S); //costruttore
    virtual ~Planner(){} //distruttore
    virtual void start(){} //inizio planning

    //ricerca
    virtual void radius(int){}

    //sampling
    int prune();
    Node* sampleFreeSpace();
    Node* sampleEllipsoid(float Cbest, float Cmin, Eigen::Vector3f x_center, Eigen::Matrix3f C);
    Eigen::Vector3f sampleUnitBall();


    //collision checking
    bool is_collision(std::pair<Node*, Node*> e);
    bool is_intersect_circle(Eigen::Vector2f o,
                             Eigen::Vector2f d,
                             Eigen::Vector2f a,
                             float r);
    bool is_intersect_rec(std::pair<Node*, Node*> e,
                                    Eigen::Vector2f o,
                                    Eigen::Vector2f d,
                                    std::pair<float, float> a,
                                    std::pair<float, float> b);
    bool is_in_obs(float x[2]);
    bool is_in_obs(Node* x);


    //ausiliari
    float dist(Node* x1, Node* x2); //distanza-nodes
    float dist(Eigen::Vector2f x1, Eigen::Vector2f x2); //distanza-vector
    float angle(Node* x1, Node* x2); //angolo
    Eigen::Matrix3f rotationToWorldFrame(Node* x1, Node* x2, float L); //matrice rotazione
    float randf(float min, float max); //genero numeri random float
    virtual void updateCbest(){} //aggiorno soluzione comune
    std::vector<std::pair<float, float>> extractPath(); //estraggo path finale


protected:
    //------ variabili -------------------------
    Node* x_start;
    Node* x_goal;
    Tree T;
    float Cshared; //miglior soluzione trovata
    float r; //reggio ricerca
    float eta; //definisco parametri ricerca
    float err; //bound aggiuntivo
    GraphWidget* widget; //widget da aggiornare

    //-------- funzioni ------------------------
    virtual bool terminationCondition(float Cmin, std::chrono::time_point<std::chrono::high_resolution_clock> t_start){}; //check termination condition
    virtual void stop(std::chrono::time_point<std::chrono::high_resolution_clock> t_start){} //stop planning (per tutti i thread)
};

#endif // PLANNER_H

