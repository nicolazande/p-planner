#include "planningdata.h" //--> poteri anche farlo con struzzi

//costruttore planning data --> specifico per ogni thread
PlanningData::PlanningData(Env* env,
                           float ext[2][2],
                           int id,
                           float stepLen,
                           float padding,
                           float dt,
                           bool  visual,
                           float t_max,
                           float m,
                           float threshold)
{
    this->env = env;
    this->x_start[0] = ext[0][0];
    this->x_start[1] = ext[0][1];
    this->x_goal[0] = ext[1][0];
    this->x_goal[1] = ext[1][1];
    this->id = id;
    this->stepLen = stepLen;
    this->padding = padding;
    this->dt = dt;
    this->visual = visual;
    this->t_max = t_max;
    this->m = m;
    this->threshold = threshold;
}



//costruttore settings
Settings::Settings(int iterations,
                   bool visualization,
                   std::string saveDir,
                   std::pair<float, float> x_start,
                   std::pair<float, float> x_goal)
{
    this->iterations = iterations;
    this->visualization = visualization;
    this->saveDir = saveDir;
    this->x_start = x_start;
    this->x_goal = x_goal;
}
