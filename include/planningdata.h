#ifndef PLANNINGDATA_H
#define PLANNINGDATA_H
#include<stdlib.h>
#include <string>
//forvard declaration
class Env;

class PlanningData
{
public:
    //-------- variabili -------------------------
    Env* env;
    float x_start[2];
    float x_goal[2];
    int id; //id --> per sapere quale thread
    float stepLen;
    float padding;
    float dt;
    bool visual;
    float t_max;
    float m; //parametro su cui giocare
    float threshold;


    //--------- funzioni -------------------------
    PlanningData(Env* env,
                 float ext[2][2],
                 int id,
                 float stepLen,
                 float padding,
                 float dt,
                 bool visual,
                 float t_max,
                 float m, // IMPORTANTE!!! -> parametro tuning
                 float threshold); //costruttore
};


//impostazioni planning
class Settings
{
public:
    //---------------- variabili -------------------------
    int iterations;
    bool visualization;
    std::string saveDir;
    std::pair<float, float> x_start;
    std::pair<float, float> x_goal;

    //---------------- funzioni --------------------------
    Settings(int iterations,
             bool visualization,
             std::string saveDir,
             std::pair<float, float> x_start,
             std::pair<float, float> x_goal);

};


#endif // PLANNINGDATA_H
