#include "env.h"
#include <iostream>

//costruttore --> definisco limiti e ostacoli
Env::Env(int item)
{
    //boundary rettangolo [minX, maxX, minY, maxY]
    this->obs_boundary << 0, 50, 0, 30;

    //Environment 0
    if(item == 0){

        //ostacolo rettangolare [minX, minY, w, h]
        this->obs_rectangle.resize(4, 4);
        this->obs_rectangle.row(0) << 13, 11,  8,  8;
        this->obs_rectangle.row(1) << 18, 22,  8,  3;
        this->obs_rectangle.row(2) << 26,  7,  2, 12;
        this->obs_rectangle.row(3) << 32, 14, 10,  2;

        //ostacolo cerchio [Xc, Yc, r]
        this->obs_circle.resize(5, 3);
        this->obs_circle.row(0) <<  7, 12,  3;
        this->obs_circle.row(1) << 44, 20,  3;
        this->obs_circle.row(2) << 15,  5,  3;
        this->obs_circle.row(3) << 37,  7,  3;
        this->obs_circle.row(4) << 37, 23,  3;
    }
    else if(item == 1){

        //ostacolo rettangolare [minX, minY, w, h]
        this->obs_rectangle.resize(15, 4);
        this->obs_rectangle.row(0)  << 13,  8,  4,  6;
        this->obs_rectangle.row(1)  << 17, 17,  3,  5;
        this->obs_rectangle.row(2)  << 22,  7,  5,  3;
        this->obs_rectangle.row(3)  << 21, 26,  4,  3;
        this->obs_rectangle.row(4)  << 29, 20,  3,  5;
        this->obs_rectangle.row(5)  << 31, 12,  7,  3;
        this->obs_rectangle.row(6)  << 24, 13,  3,  7;
        this->obs_rectangle.row(7)  << 11, 23,  5,  3;
        this->obs_rectangle.row(8)  << 11,  1,  5,  3;
        this->obs_rectangle.row(9)  << 23,  2,  4,  4;
        this->obs_rectangle.row(10) <<  2, 26,  3,  3;
        this->obs_rectangle.row(11) << 36,  1,  3,  3;
        this->obs_rectangle.row(12) << 40, 25,  2,  2;
        this->obs_rectangle.row(13) <<  2, 11,  3,  3;
        this->obs_rectangle.row(14) << 46,  2,  3,  3;

        //ostacolo cerchio [Xc, Yc, r]
        this->obs_circle.resize(9, 3);
        this->obs_circle.row(0) << 34, 23, 4;
        this->obs_circle.row(1) << 30,  4, 4;
        this->obs_circle.row(2) <<  5,  5, 4;
        this->obs_circle.row(3) <<  5, 15, 4;
        this->obs_circle.row(4) <<  6, 22, 4;
        this->obs_circle.row(5) << 37, 17, 4;
        this->obs_circle.row(6) << 42,  7, 4;
        this->obs_circle.row(7) << 45, 13, 4;
        this->obs_circle.row(8) << 45, 25, 4;
    }
}
