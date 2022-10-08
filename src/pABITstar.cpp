#include "pABITstar.h"

//============= pABITstar ==================================

//costruttore --> stesso di planner
pABITstar::pABITstar(GraphWidget* widget, PlanningData* PD, Solution* S)
    : Planner(widget, PD, S)
{

}


//distruttore --> pulisce albero e unconnected
pABITstar::~pABITstar()
{
    //unisco i set per non averne doppi
    this->T.V.insert(this->T.Qv.begin(), this->T.Qv.end());

    //elimino nodi in (V U Qv) + dealloco memoria
    for(auto i : this->T.V){
        delete i;
    }

    //elimino resto
    this->T.V.clear();
    this->T.Qv.clear();
    this->T.E.clear();
    this->T.Qe.clear();
}


//lancio il pABITstar
void pABITstar::start(){

    //faccio partire timer
    auto t_start = std::chrono::high_resolution_clock::now();

    //inizializzo Tree, Xunconnected e dizionari
    this->T.V.insert(this->x_start);
    this->T.V.insert(this->x_goal);
    this->Xunconn.insert(this->x_goal);
    this->gT[this->x_start] = 0;
    this->gT[this->x_goal] = INF;

    //aggiorno parametri [cMin, theta, xCenter, C] che definiscono ovale di ricerca
    float Cmin = this->dist(this->x_start, this->x_goal); //costo minimo soluzione
    Eigen::Matrix3f C = this->rotationToWorldFrame(this->x_start, this->x_goal, Cmin);
    Eigen::Vector3f x_center((this->x_start->S[0] + this->x_goal->S[0])/2,
                             (this->x_start->S[1] + this->x_goal->S[1])/2,
                             0);

    int dm = 0; //delta samples (durante pruning)
    float eps_prune = 0.f; //[1 = sample costanti, 0 = sommo a ogli resampling]
    long int i = 0; //indice ricerca
    std::set<Node*> newSamples; //set ausiliario per resampling
    std::set<std::pair<Node*, Node*>>::iterator ie; //iteratore edges
    Node* bv = nullptr; //node pointer per evitare di rimanere bloccato in expandvertex

    //inizio ciclo di pABITstar
    while(!this->terminationCondition(Cmin, t_start)){

        //controllo se Qe e Qv sono vuote
        if(this->T.Qe.empty() && this->T.Qv.empty()){
            //pruning albero e set disconnesso (dopo prima soluzione)
            if(this->gTf(this->x_goal) < INF){
                dm = this->prune();
            }
            //nuovo resample, tiene conto dell' ultimo pruning
            newSamples = sample(static_cast<int>(this->PD->m - eps_prune*dm),
                                this->gTf(this->x_goal), Cmin, x_center, C);
            //unisco a unconnected nuovi samples
            this->Xunconn.insert(newSamples.begin(), newSamples.end());
            newSamples.clear();
            //salvo Vertici vecchi per confronto (copia)
            this->T.Vold = this->T.V;
            //inizializzo nuova coda di vertici (copia)
            this->T.Qv = this->T.V;
            //riduco eta solo dopo prima soluzione
            if(this->gTf(this->x_goal) < INF){
                this->eta = 5;
            }
            //uso formula raggio solo dopo primo sampling
            if(i>0)
                this->radius(static_cast<int>(this->T.V.size()) + static_cast<int>(this->Xunconn.size()));
        }

        //costruisco tree con edge in ordine di costo
        while(this->bestVertexQueueValue() <= this->bestEdgeQueueValue()){
            bv = this->bestInVertexQueue();
            if(bv == nullptr)
                break;
            this->expandVertex(bv);
        }

        //se Qe = empty --> nuovo sampling (SICURA per non bloccarmi)
        if(this->T.Qe.empty()){
            eps_prune = 0; //voglio sommare samples
            continue;
        }

        //miglior edge possibile
        std::pair<Node*, Node*> e = this->bestInEdgeQueue();
        this->T.Qe.erase(e);

        //controllo se edge puo migliorare soluzione corrente
        if(this->gTf(e.first) + this->dist(e.first, e.second) + this->h_estimated(e.second) < this->gTf(this->x_goal)){
            float actual_cost = this->cost(e);
            if(this->g_estimated(e.first) + actual_cost + this->h_estimated(e.second) < this->gTf(this->x_goal)){
                if(this->gTf(e.first) + actual_cost < this->gTf(e.second)){
                    //edge gia in albero --> rewiring
                    if(this->T.V.count(e.second) > 0){
                        ie = this->T.E.begin();
                        while(ie!=this->T.E.end()){
                            if(ie->second == e.second){
                                ie = this->T.E.erase(ie);
                            }
                            else
                                ie++;
                        }
                    }
                    //edge non presente --> expansion
                    else {
                        this->Xunconn.erase(e.second);
                        this->T.V.insert(e.second);
                        this->T.Qv.insert(e.second);
                    }

                    this->gT[e.second] = this->gTf(e.first) + actual_cost;
                    this->T.E.insert(e);
                    e.second->parent = e.first;

                    ie = this->T.Qe.begin();
                    while(ie!=this->T.Qe.end()){
                        if(ie->second == e.second &&
                           this->gTf(ie->first) + this->dist(ie->first, ie->second) >= this->gTf(e.second)){
                            ie = this->T.Qe.erase(ie);
                        }
                        else
                            ie++;
                    }
                }
            }
        }
        else {
            this->T.Qe.clear();
            this->T.Qv.clear();
        }
        //provo a aggiornare soluzione condivisa
        this->updateCbest();
        i++;
    }
    //fine pABITstar --> termination conditions soddisfatte
    this->stop(t_start);
}



//--------- funzioni per ricerca ----------------------------------------------

//dato un vertice creo i possibili vertici
void pABITstar::expandVertex(Node* v){

    //sicura per nullptr
    if(v == nullptr)
        return;

    //tolgo v da coda vertici --> inizio a processarlo
    this->T.Qv.erase(v);

    //trovo vicini di v in Xunconn
    std::set<Node*> Xnear;
    for(auto x : this->Xunconn){
        if(this->dist(x, v) < this->r)
            Xnear.insert(x);
    }

    //aggiungo edge a coda Qe e aggiorno vocabolario
    for(auto x : Xnear){
        if(this->g_estimated(v) + this->dist(v, x) + this->h_estimated(x) < this->gTf(this->x_goal)){
            this->gT[x] = INF;
            this->T.Qe.insert({v, x});
        }
    }

    //procedo solo se sample nuovo (non in albero vecchio)
    std::set<Node*> Vnear;
    if(this->T.Vold.count(v)==0){
        //trovo vicini di v in V (gia connessi)
        for(auto w: this->T.V){
            if(this->dist(w, v) <= this->r)
                Vnear.insert(w);
        }

        //espando vertici --> inserisco quelli promettenti
        for(auto w : Vnear){
            if(this->T.E.count({v, w}) == 0 &&
               this->g_estimated(v) + this->dist(v, w) + this->h_estimated(w) < this->gTf(this->x_goal) &&
               this->gTf(v) + this->dist(v, w) < this->gTf(w)){
                //aggiungo edge all'albero
                this->T.E.insert({v, w});
                //se w = nodo nuovo --> aggiungo a dizionario
                if(this->gT.count(w) == 0)
                    this->gT[w] = INF;
            }
        }
    }
}


//trovo vertice migliore in coda (unica ricerca)
Node* pABITstar::bestInVertexQueue(){
    float min = INF;
    float min_tmp;
    std::set<Node*>::iterator it, best_vertex;

    //controllo se ho in coda vertici
    if(!this->T.Qv.empty()){
        it = this->T.Qv.begin();
        best_vertex = it;
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.Qv.end()){
            min_tmp = this->gTf(*it) + this->h_estimated(*it);
            if(min_tmp < min){
                min = min_tmp;
                best_vertex = it;
            }
            it++;
        }
    }
    //se coda vuota ritorno nullptr
    else{
        std::cout << "No vertexes in queue Qv" << std::endl;
        return nullptr;
    }
    return *best_vertex;
}


//trovo edge migliore in coda (unica ricerca)
std::pair<Node*, Node*> pABITstar::bestInEdgeQueue(){
    float min = INF;
    float min_tmp;
    std::set<std::pair<Node*, Node*>>::iterator it, best_vertex;

    //controllo se ho in coda vertici
    if(!this->T.Qe.empty()){
        it = this->T.Qe.begin();
        best_vertex = it;
        //ciclo su vertex e tengo traccia del migliore
        while(it!=this->T.Qe.end()){
            min_tmp = this->gTf(it->first) + this->dist(it->first, it->second) + this->h_estimated(it->second);
            if(min_tmp < min){
                min = min_tmp;
                best_vertex = it;
            }
            it++;
        }
    }
    //se coda vuota ritorno nullptr
    else{
        std::cout << "No edges in queue Qe" << std::endl;
        return {nullptr, nullptr};
    }
    return *best_vertex;
}


//trovo valore vertice meno costoso in coda
float pABITstar::bestVertexQueueValue(){
    float min = INF;
    float min_tmp;
    if(!this->T.Qv.empty())
        for(auto v : this->T.Qv){
            min_tmp = this->gTf(v) + this->h_estimated(v);
            if(min_tmp < min)
                min = min_tmp;
        }
    return min;
}


//trovo valore edge meno costoso in coda
float pABITstar::bestEdgeQueueValue(){
    float min = INF;
    float min_tmp;
    if(!this->T.Qe.empty())
        for(auto e : this->T.Qe){
            min_tmp = this->gTf(e.first) + this->dist(e.first, e.second) + this->h_estimated(e.second);
            if(min_tmp < min)
                min = min_tmp;
        }
    return min;
}


//--------- euristiche e stime costi ------------------------------------------

//ritorno cost to come a nodo nel dizionario
float pABITstar::gTf(Node* x){
    //non nel dizionario --> disconnesso = INF
    if(gT.count(x)==0)
        return INF;
    return gT[x];
}


//stima costo totale percorso
float pABITstar::f_estimated(Node* x){
    return this->g_estimated(x) + h_estimated(x);
}


//stima cost to come
float pABITstar::g_estimated(Node* x){
    return this->dist(this->x_start, x);
}


//stima cost to go
float pABITstar::h_estimated(Node* x){
    return this->dist(x, this->x_goal);
}


//--------- funzioni per sampling  e pruning -----------------------------------

//pruning
int pABITstar::prune(){
    //elimino samples inutili da X_unconn
    std::set<Node*>::iterator it = this->Xunconn.begin();
    while(it!=this->Xunconn.end()){
        // ">=" --> tenso solo potenzialmente migliori
        if(this->f_estimated(*it) >= this->gTf(this->x_goal)){
            delete *it; //libero memoria = nodo
            it = this->Xunconn.erase(it); //elimino item in set
        }
        else
            it++;
    }
    //elimino nodi che non migliorano soluzione da V
    it = this->T.V.begin();
    while(it!=this->T.V.end()){
        //">" --> tengo migliori o uguali
        if(this->f_estimated(*it) > this->gTf(this->x_goal)){
            delete *it;
            it = this->T.V.erase(it);
        }
        else
            it++;
    }
    //elimino gli edge inutili da E
    std::set<std::pair<Node*, Node*>>::iterator ie = this->T.E.begin();
    while(ie!=this->T.E.end()){
        //non libero memoria --> tengo i nodi e elimino solo edge
        if(this->f_estimated(ie->first) > this->gTf(this->x_goal) || this->f_estimated(ie->second) > this->gTf(this->x_goal)){
            ie = this->T.E.erase(ie);
        }
        else
            ie++;
    }
    //metto samples disconnessi di V in X_unconn
    it = this->T.V.begin();
    while(it!=this->T.V.end()){
        if(this->gTf(*it)>=INF){
            this->Xunconn.insert(*it);
            it = this->T.V.erase(it);
        }
        else
            it++;
    }
    //ritorno grandezza set, utile per nuovo batch
    return static_cast<int>(this->Xunconn.size());
}


//sampling overload
std::set<Node*> pABITstar::sample(int m, float Cbest, float Cmin, Eigen::Vector3f x_center, Eigen::Matrix3f C){
    if(Cbest < INF)
        return this->sampleEllipsoid(m, Cbest, Cmin, x_center, C);
    else
        return this->sampleFreeSpace(m);
}


//sampling iniziale nel free space overload
std::set<Node*> pABITstar::sampleFreeSpace(int m){
    std::set<Node*> Batch;
    int ind = 0;
    //limiti state space
    float Xmin = this->PD->env->obs_boundary[0] + this->err;
    float Xmax = this->PD->env->obs_boundary[1] - this->err;
    float Ymin = this->PD->env->obs_boundary[2] + this->err;
    float Ymax = this->PD->env->obs_boundary[3] - this->err;
    //estraggo m samples (non in collisione)
    while(ind < m){
        Node* node = new Node(randf(Xmin, Xmax), randf(Ymin, Ymax));
        //scarto nodo
        if(this->is_in_obs(node))
            continue;
        //aggiungo nodo
        else{
            Batch.insert(node);
            ind++;}
    }
    return Batch;
}


//sampling all'interno ellissoide di ricerca overload
std::set<Node*> pABITstar::sampleEllipsoid(int m, float Cbest, float Cmin, Eigen::Vector3f x_center, Eigen::Matrix3f C){
    float r = static_cast<float>(std::sqrt(std::abs(std::pow(Cbest, 2) + std::pow(Cmin, 2)))) / 2;
    Eigen::DiagonalMatrix<float, 3> L(Cbest/2, r, r);
    int ind = 0;
    std::set<Node*> Batch;
    Eigen::Vector3f xBall;
    Eigen::Vector3f x_rand;

    while(ind<m){
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
            Batch.insert(new Node(x_rand(0), x_rand(1)));
            ind++;
        }
    }
    return Batch;
}


//sampling in un cerchio unitario overload
Eigen::Vector3f pABITstar::sampleUnitBall(){
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



//-------------- funzioni ausiliarie -------------------------------

//vero costo di un edge
float pABITstar::cost(std::pair<Node*, Node*> e){
    if(this->is_collision(e))
        return INF;
    return this->dist(e.first, e.second);
}


//raggio di ricerca
void pABITstar::radius(int q){
    int l = 0;
    for(auto v : this->T.V)
        if(this->f_estimated(v) <= this->gTf(this->x_goal))
            l++;
    this->r = 2 * this->eta * static_cast<float>(sqrt(1.5 * l / 3.14 * log(q) / q));
}


//aggiorno soluzione comune
void pABITstar::updateCbest(){
    //se migliorato --> aggiorno soluzione comune
    if(this->gTf(this->x_goal) < this->S->Cbest){
        this->S->Cbest = this->gTf(this->x_goal);
        std::cout << std::endl << "new Cbest = " << this->S->Cbest <<
                     ", found by thread" << this->PD->id << std::endl << std::endl;
    }
}


//check termination condition
bool pABITstar::terminationCondition(float Cmin, std::chrono::time_point<std::chrono::high_resolution_clock> t_start){
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
    return ((this->S->state == true && this->PD->visual) || this->gTf(this->x_goal) <= this->threshold * Cmin ||
            static_cast<float>(duration.count() / 1e6) > this->PD->t_max);
}


//stop: fine pABITstar per tutti i thread
void pABITstar::stop(std::chrono::time_point<std::chrono::high_resolution_clock> t_start){
    //il primo thread stampa a viseo e accende flag
    auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);

    //test visual e truncation
    if(!this->S->state && this->PD->visual){
        if(this->gTf(this->x_goal) < INF){
            //-------- aggiorno soluzione e aggiungo risultati -------------------
            this->S->time =  static_cast<float>(duration.count() / 1e6);
            this->S->winner = "pABIT[" + std::to_string(this->PD->id) + "]";
        }
        else {
            this->S->winner = "None";
            this->S->time =  INF;
        }
        //segno fine ricerca
        this->S->state = true;
        //stampo soluzione finale
        if(this->gTf(this->x_goal)<INF && this->PD->visual){
            this->widget->buildPath(this->extractPath());
        }
    }

    //benchmarking --> non spengo flag e aspetto tutti
    if(!this->PD->visual){
        this->S->results_vec.push_back({10+this->PD->id, {static_cast<float>(duration.count() / 1e3), this->gTf(this->x_goal)}});
    }
}
