#ifndef ENV_H
#define ENV_H
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class Env
{
public:
    //--------- variabili -----------------------
    Eigen::Matrix<float, 1, 4> obs_boundary;

    //environment0
    Eigen::MatrixX4f obs_rectangle;
    Eigen::MatrixX3f obs_circle;

    //--------- funzioni ------------------------
    Env(int item); //costruttore
};

#endif // ENV_H
