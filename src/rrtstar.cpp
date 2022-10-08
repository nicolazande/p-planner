#include "rrtstar.h"

//============== funzioni base ====================================

//costruttore --> stesso di planner
RRTstar::RRTstar(GraphWidget* widget, PlanningData* PD, Solution* S)
    : Planner(widget, PD, S)
{
    //inizializzo parametri aggiuntivi
    this->r = 20;
    //connetto x_start
    this->T.V.insert(this->x_start);
}


//distruttore --> pulisco tutto
RRTstar::~RRTstar(){
    //unisco i set per non averne doppi
    this->T.V.insert(this->T.Qv.begin(), this->T.Qv.end());
    //elimino nodi in (V U Qv) + dealloco memoria
    for(auto i : this->T.V){
        delete i;
    }
    this->T.V.clear();
    this->T.Qv.clear();
}


//lancio RRTstar
void RRTstar::start(){
    //faccio partire timer
    auto t_start = std::chrono::high_resolution_clock::now();
    int i = 0;
    Node* q_rand = nullptr;
    Node* q_near = nullptr;
    Node* q_new = nullptr;
    //minimo costo raggiungibile
    float Cmin = this->dist(this->x_start, this->x_goal);

    //ciclo fino a termination condition
    while(!this->terminationCondition(Cmin, t_start)){
        q_rand = this->sample(); //sampling
        q_near = this->nearestNeighbour(q_rand); //nearest neighbour
        q_new = this->steer(q_near, q_rand); //steering
        //controllo collisione --> ottimizzo
        if(!this->is_collision({q_near, q_new})){
            this->findNeighbours(q_new); //vicini
            this->T.V.insert(q_new);
            //rewiring
            if(!this->T.Qv.empty()){
                this->choseParent(q_new);
                this->rewire(q_new);
            }
        }
        //cerco di connettere x_goal (non sempre --> piu veloce)
        if(i%10 == 0)
            this->searchGoalParent();
        i++;
    }
    //fermo e pubblico
    this->stop(t_start);
}




//============== funzioni sampling ============================
//sampling
Node* RRTstar::sample(){
    if(this->randf(0, 1) < this->PD->m){
        return new Node(this->x_goal->S[0],
                        this->x_goal->S[1]);
    }
    else
        return this->sampleFreeSpace();
}




//============== costo =======================================

//costo di un vertice
float RRTstar::cost(Node* q){
    float tot = 0;
    while(q->parent != nullptr){
        tot += this->dist(q, q->parent);
        if(q->parent == this->x_start){
            return tot;
        }
        q = q->parent;
    }
    return INF;
}




//================ funzioni di ricerca ========================

//search goal parent
void RRTstar::searchGoalParent(){
    float min = INF;
    float min_tmp;
    std::set<Node*>::iterator it;
    float d;
    //controllo se ho in coda vertici
    if(!this->T.V.empty()){
        it = this->T.V.begin();
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.V.end()){
            d = dist(*it, this->x_goal);
            min_tmp = this->cost(*it) + d;
            if(d <= this->PD->stepLen){
                if( min_tmp < min){
                    min = min_tmp;
                    this->x_goal->parent = *it;
                }
            }
            it++;
        }
    }
    //check se aggiornare costo soluzione
    float Ccurrent = this->cost(this->x_goal);
    if(Ccurrent < this->S->Cbest){
        this->S->Cbest = Ccurrent;
    }
}


//rewire albero
void RRTstar::rewire(Node* q){
    std::set<Node*>::iterator it;
    if(!this->T.Qv.empty()){
        it = this->T.Qv.begin();
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.Qv.end()){
            if(this->cost(*it) > this->cost(q) + this->dist(q, *it)){
                (*it)->parent = q;
            }
            it++;
        }
    }
}


//trovo miglior vicino
void RRTstar::findNeighbours(Node* q_new){
    this->T.Qv.clear();
    float n = this->T.V.size()+1;
    float r = std::min(static_cast<float>(this->r * std::sqrt(log(n)/n)), this->PD->stepLen); //clampo raggio ricerca
    std::set<Node*>::iterator it;
    //controllo se ho in coda vertici
    if(!this->T.V.empty()){
        it = this->T.V.begin();
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.V.end()){
            //se sono nel raggio di ricerca aggiungo in coda
            if(dist(q_new, *it) <= r){
                this->T.Qv.insert(*it);
            }
            it++;
        }
    }
}


//trovo nearest neighbour
Node* RRTstar::nearestNeighbour(Node* x){
    float min = INF;
    float min_tmp;
    std::set<Node*>::iterator it, nn;
    //controllo se ho in coda vertici
    if(!this->T.V.empty()){
        it = this->T.V.begin();
        nn = it;
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.V.end()){
            min_tmp = this->dist(x, *it);
            if(min_tmp < min){
                min = min_tmp;
                nn = it;
            }
            it++;
        }
    }
    //se coda vuota ritorno nullptr
    else{
        std::cout << "No vertexes in queue V" << std::endl;
        return nullptr;
    }
    return *nn;
}


//trovo miglior parent per rewiring
void RRTstar::choseParent(Node* q){
    //non assegno mai a start parent
    if(q == this->x_start)
        return;
    float min = INF;
    float min_tmp;
    std::set<Node*>::iterator it, parent;
    //controllo se ho in coda vertici (altrimenti gia nullptr)
    if(!this->T.Qv.empty()){
        it = this->T.Qv.begin();
        parent = it;
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.Qv.end()){
            //costo cumulativo per raggiungere q
            min_tmp = this->cost(*it) + this->dist(*it, q);
            if(min_tmp < min){
                min = min_tmp;
                parent = it;
            }
            it++;
        }
        //assegno parent a q
        q->parent = *parent;
    }
}




//============== generale =================================

//steering function
Node* RRTstar::steer(Node* q_near, Node* q_rand){
    float d = std::min(this->dist(q_near, q_rand), this->PD->stepLen);
    float a = this->angle(q_near, q_rand);
    Node* q_new = new Node(q_near->S[0] + d * std::cos(a),
                              q_near->S[1] + d * std::sin(a));
    q_new->parent = q_near;
    delete q_rand;
    return q_new;
}


//controllo se esiste una soluzione
bool RRTstar::isFound(Node* q){
    while(q->parent != nullptr){
        if(q->parent == this->x_start)
            return true;
        q = q->parent;
    }
    return false;
}


//check termination condition
bool RRTstar::terminationCondition(float Cmin, std::chrono::time_point<std::chrono::high_resolution_clock> t_start){
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    return ((this->S->state == true && this->PD->visual) || this->cost(this->x_goal) <= this->threshold * Cmin ||
            static_cast<float>(duration.count() / 1e6) > this->PD->t_max);
}


//stop: fine RRTstar per tutti i thread
void RRTstar::stop(std::chrono::time_point<std::chrono::high_resolution_clock> t_start){
    //il primo thread stampa a viseo e accende flag
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

    //uscita per test
    if(!this->S->state && this->PD->visual){
        if(this->S->Cbest < INF){
            //-------- aggiorno soluzione e aggiungo risultati -------------------
            this->S->time =  static_cast<float>(duration.count() / 1e6);
            this->S->winner = "pRRT[" + std::to_string(this->PD->id) + "]";
        }
        else {
            this->S->winner = "None";
            this->S->time =  INF;
        }
        //segno fine ricerca
        this->S->state = true;
        //stampo soluzione finale
        if(this->S->Cbest < INF && this->PD->visual){
            this->widget->buildPath(this->extractPath());
        }
    }

    //benchmarking --> non spengo flag e aspetto tutti
    if(!this->PD->visual){
        this->S->results_vec.push_back({20+this->PD->id, {static_cast<float>(duration.count() / 1e3), this->cost(this->x_goal)}});
    }
}



