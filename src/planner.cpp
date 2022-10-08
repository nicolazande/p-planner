#include "planner.h"


//============= pABITstar ==================================

//costruttore
Planner::Planner(GraphWidget* widget, PlanningData* PD, Solution* S){
    this->PD = PD;
    this->S = S;
    this->x_start = new Node(this->PD->x_start[0], this->PD->x_start[1]);
    this->x_goal =  new Node(this->PD->x_goal[0], this->PD->x_goal[1]);
    this->r = 15;
    this->eta = 7;
    this->threshold = PD->threshold;
    this->err = PD->padding;
    this->widget = widget;
}


//--------- funzioni per sampling  e pruning -----------------------------------

//sampling iniziale nel free space
Node* Planner::sampleFreeSpace(){
    Node* node;

    //limiti state space
    float Xmin = this->PD->env->obs_boundary[0] + this->err;
    float Xmax = this->PD->env->obs_boundary[1] - this->err;
    float Ymin = this->PD->env->obs_boundary[2] + this->err;
    float Ymax = this->PD->env->obs_boundary[3] - this->err;

    //estraggo m samples (non in collisione)
    while(true){
        node = new Node(randf(Xmin, Xmax), randf(Ymin, Ymax));
        //scarto nodo
        if(!this->is_in_obs(node))
            break;
    }
    return node;
}


//sampling all'interno ellissoide di ricerca
Node* Planner::sampleEllipsoid(float Cbest, float Cmin, Eigen::Vector3f x_center, Eigen::Matrix3f C){

    float r = static_cast<float>(std::sqrt(std::abs(std::pow(Cbest, 2) + std::pow(Cmin, 2)))) / 2;
    Eigen::DiagonalMatrix<float, 3> L(Cbest/2, r, r);
    Node* node;
    std::set<Node*> Batch;
    Eigen::Vector3f xBall;
    Eigen::Vector3f x_rand;

    while(true){
        xBall = this->sampleUnitBall();
        x_rand = (C.transpose()*L).transpose()*xBall + x_center;
        //controllo se sono nei limiti (in teoria non serve ma in pratica...)
        if(x_rand(0) < this->PD->env->obs_boundary(0) + this->err)
            continue;
        if(x_rand(0) > this->PD->env->obs_boundary(1) - this->err)
            continue;
        if(x_rand(1) < this->PD->env->obs_boundary(2) + this->err)
            continue;
        if(x_rand(1) > this->PD->env->obs_boundary(3) - this->err)
            continue;
        //check se in ostacolo (uso coordinate per non crare nodo se non necessario)
        float coord[2] = {x_rand(0), x_rand(1)};
        if(!this->is_in_obs(coord)){
            node = new Node(x_rand(0), x_rand(1));
            break;
        }
    }
    return node;
}


//sampling in un cerchio unitario
Eigen::Vector3f Planner::sampleUnitBall(){
    Eigen::Vector3f xBall;
    while(true){
        float x = this->randf(-1, 1);
        float y = this->randf(-1, 1);
        if(pow(x, 2) + pow(y,2) < 1){
            xBall(0) = x;
            xBall(1) = y;
            xBall(2) = 0;
            return xBall;
        }
    }
}


//-------------- collision checking --------------------------------

//controllo se dentro ostacolo (overload float)
bool Planner::is_in_obs(float x[2]){

    //controllo se sample interno a ostacoli cerchi
    auto r = this->PD->env->obs_circle.rows();
    for (int i = 0; i < r; i++) {
        if(std::sqrt(pow(x[0] - this->PD->env->obs_circle(i, 0), 2) +
                     pow(x[1] - this->PD->env->obs_circle(i, 1), 2)) < this->PD->env->obs_circle(i, 2) + this->err)
            return true;
    }

    //controllo se sample interno a ostacoli rettangolari
    r = this->PD->env->obs_rectangle.rows();
    for (auto i = 0; i < r; i++) {
        if(x[0] - this->PD->env->obs_rectangle(i, 0) >=  -this->err &&
           x[0] - this->PD->env->obs_rectangle(i, 0) <= this->PD->env->obs_rectangle(i, 2) + this->err &&
           x[1] - this->PD->env->obs_rectangle(i, 1) >= -this->err &&
           x[1] - this->PD->env->obs_rectangle(i, 1) <= this->PD->env->obs_rectangle(i, 3) + this->err)
            return true;
    }
    return false;
}


//controllo se dentro ostacolo (overload Node*)
bool Planner::is_in_obs(Node* x){

    //controllo se sample interno a ostacoli cerchi
    auto r = this->PD->env->obs_circle.rows();
    for (auto i = 0; i < r; i++) {
        if(std::sqrt(pow(x->S[0] - this->PD->env->obs_circle(i, 0), 2) +
                     pow(x->S[1] - this->PD->env->obs_circle(i, 1), 2)) <= this->PD->env->obs_circle(i, 2) + this->err)
            return true;
    }

    //controllo se sample interno a ostacoli rettangolari
    r = this->PD->env->obs_rectangle.rows();
    for (auto i = 0; i < r; i++) {
        if(x->S[0] - this->PD->env->obs_rectangle(i, 0) >= -this->err &&
           x->S[0] - this->PD->env->obs_rectangle(i, 0) <= this->PD->env->obs_rectangle(i, 2) + this->err &&
           x->S[1] - this->PD->env->obs_rectangle(i, 1) >= -this->err &&
           x->S[1] - this->PD->env->obs_rectangle(i, 1) <= this->PD->env->obs_rectangle(i, 3) + this->err)
            return true;
    }
    return false;
}


//controllo collsione generale edge
bool Planner::is_collision(std::pair<Node*, Node*> e){

    //se i vertici sono in un ostacoli --> collisione
    if(this->is_in_obs(e.first) || this->is_in_obs(e.second))
        return true;

    //controllo se interseca ostacoli rettangolari
    Eigen::Vector2f o(e.first->S[0], e.first->S[1]);
    Eigen::Vector2f d(e.second->S[0] - e.first->S[0], e.second->S[1] - e.first->S[1]);

    //trovo vertici di ostacoli rettangolari
    auto r = this->PD->env->obs_rectangle.rows();
    Eigen::MatrixXf vert(r, 8);
    Eigen::VectorXf pad = Eigen::VectorXf::Ones(r) * this->err;
    vert.col(0) = this->PD->env->obs_rectangle.col(0) - pad;
    vert.col(1) = this->PD->env->obs_rectangle.col(1) - pad;
    vert.col(2) = this->PD->env->obs_rectangle.col(0) + this->PD->env->obs_rectangle.col(2) + pad;
    vert.col(3) = this->PD->env->obs_rectangle.col(1) - pad;
    vert.col(4) = this->PD->env->obs_rectangle.col(0) + this->PD->env->obs_rectangle.col(2) + pad;
    vert.col(5) = this->PD->env->obs_rectangle.col(1) + this->PD->env->obs_rectangle.col(3) + pad;
    vert.col(6) = this->PD->env->obs_rectangle.col(0) - pad;
    vert.col(7) = this->PD->env->obs_rectangle.col(1) + this->PD->env->obs_rectangle.col(3) + pad;

    //controllo intersezione con vertici (continuo)
    for (auto i=0; i<r; i++){
        if(this->is_intersect_rec(e, o, d, {vert(i,0), vert(i, 1)}, {vert(i,2), vert(i,3)}))
            return true;
        if(this->is_intersect_rec(e, o, d, {vert(i,2), vert(i, 3)}, {vert(i,4), vert(i,5)}))
            return true;
        if(this->is_intersect_rec(e, o, d, {vert(i,4), vert(i, 5)}, {vert(i,6), vert(i,7)}))
            return true;
        if(this->is_intersect_rec(e, o, d, {vert(i,6), vert(i, 7)}, {vert(i,0), vert(i,1)}))
            return true;
    }

    //controllo se interseco ostacoli cerch
    r = this->PD->env->obs_circle.rows();
    for (auto i=0; i<r; i++){
        if(this->is_intersect_circle(o, d, Eigen::Vector2f(this->PD->env->obs_circle(i, 0),
                                                           this->PD->env->obs_circle(i, 1)),
                                           this->PD->env->obs_circle(i, 2)))
            return true;
    }
    return false;
}


//controllo se un edge interseca ostacolo cerchio
bool Planner::is_intersect_rec(std::pair<Node*, Node*> e,
                                Eigen::Vector2f o,
                                Eigen::Vector2f d,
                                std::pair<float, float> a,
                                std::pair<float, float> b){

    //trovo vettori v1, v2, v3
    Eigen::Vector2f v1(o(0)-a.first, o(1)-a.second);
    Eigen::Vector2f v2(b.first-a.first, b.second-a.second);
    Eigen::Vector2f v3(-d(1), d(0));
    float div = v2.dot(v3);

    //se perpendicolari --> no collision
    if(div==0)
        return false;

    //controllo sovrapposizione
    float t1 = (v2(0)*v1(1) - v1(0)*v2(1)) / div; //cross product 2d a mano
    float t2 = v1.dot(v3) / div;
    if(t1>=0 && t2>=0 && t2<=1){
        Eigen::Vector2f shot(o(0)+t1*d(0), o(1)+t1*d(1));
        Eigen::Vector2f start(e.first->S[0], e.first->S[1]);
        float dist_obs = this->dist(start, shot);
        float dist_seg = this->dist(e.first, e.second);
        if(dist_obs<=dist_seg)
            return true;
    }
    return false;
}


//controllo se un edge interseca ostacolo cerchio
bool Planner::is_intersect_circle(Eigen::Vector2f o,
                                  Eigen::Vector2f d,
                                  Eigen::Vector2f a,
                                  float r){
    float d2 = d.dot(d);

    //se perpendicolare --> no collisione
    if(d2==0)
        return false;

    //bo sulla stessa riga non va
    float t = Eigen::Vector2f(a(0) - o(0), a(1)-o(1)).dot(d) / d2;

    //controllo sovrapposizione
    if(t>=0 && t<=1){
        Eigen::Vector2f shot(o(0)+t*d(0), o(1)+t*d(1));
        if(this->dist(shot, a) <= r + this->err)
            return true;
    }
    return false;
}



//-------------- funzioni ausiliarie -------------------------------

//calcolo distanza tra configurazioni (overload Node*)
float Planner::dist(Node* x1, Node* x2){
    float dx = x2->S[0] - x1->S[0];
    float dy = x2->S[1] - x1->S[1];
    return static_cast<float>(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)));
}


//calcolo distanza tra configurazioni (overload std::pair<float, float>)
float Planner::dist(Eigen::Vector2f x1, Eigen::Vector2f x2){
    float dx = x2(0) - x1(0);
    float dy = x2(1) - x1(1);
    return static_cast<float>(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)));
}


//calcolo angolo tra configurazioni
float Planner::angle(Node* x1, Node* x2){
    float dx = x2->S[0] - x1->S[0];
    float dy = x2->S[1] - x1->S[1];
    return std::atan2(dy, dx);
}


//trovo matrice di rotazione
Eigen::Matrix3f Planner::rotationToWorldFrame(Node* x1, Node* x2, float L){
    Eigen::Vector3f a1((x2->S[0] - x1->S[0])/L, (x2->S[1] - x1->S[1])/L, 0);
    Eigen::Vector3f e1(1, 0, 0);
    //creo matrice M per applicare SVD
    Eigen::Matrix3f M = a1 * e1.transpose();
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //estraggo U e V
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();
    //tromo matrice di rotazione C
    Eigen::DiagonalMatrix<float, 3> D(1, 1, U.determinant()*V.determinant());
    Eigen::Matrix3f C = U*D*V;
    return C;
}


//genero random float numbers tra min e max
float Planner::randf(float min, float max){
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}


//estraggo path finale
std::vector<std::pair<float, float>> Planner::extractPath(){
    std::vector<std::pair<float, float>> solution;
    Node* v = this->x_goal;

    //inserisco sicura per fermarmi
    this->x_start->parent = nullptr;

    //creo iterativamente la soluzione
    while(v != this->x_start){
        solution.push_back({v->S[0], v->S[1]});
        v = v->parent;
    }
    solution.push_back({this->x_start->S[0], this->x_start->S[1]});
    return solution;
}



