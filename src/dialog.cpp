#include "dialog.h"

//costruttore
Dialog::Dialog(QWidget *parent, Env* env,
               std::vector<PlanningData*> PDs1,
               std::vector<PlanningData*> PDs2,
               Settings* sett) :
    QDialog(parent), ui(new Ui::Dialog()){

    //setup dialog
    ui->setupUi(this);

    //setup parametri standard, poi li cambio
    this->env = env; //environmet
    this->PDs1 = PDs1; //planning data ABIT
    this->PDs2 = PDs2; //planning data RRT
    this->sett = sett; //settings

    //creo finestre di planning
    QVBoxLayout* base_layout = new QVBoxLayout(this);
    QHBoxLayout* menu_layout = new QHBoxLayout();
    QHBoxLayout* message_layout = new QHBoxLayout();

    //setto layouts e widgets
    base_layout->addLayout(message_layout);
    message_layout->addLayout(menu_layout);

    //creo finestre + inizializzo
    this->createWindows();

    //assegno layer ...
    this->ui->costEdit->setText("Cost: ");
    this->ui->timeEdit->setText("Time: ");
    this->ui->winnerEdit->setText("Winner: ");
    menu_layout->addWidget(this->ui->Run_Botton, 0);
    menu_layout->addWidget(this->ui->Route_Button, 1);
    message_layout->addWidget(this->ui->costEdit, 0);
    message_layout->addWidget(this->ui->timeEdit, 1);
    message_layout->addWidget(this->ui->winnerEdit, 2);
    base_layout->addWidget(this->panel);
    this->setLayout(base_layout);

    //setto iniziale e finale partendo da form senza disegno
    this->windows[0]->x_start = this->sett->x_start;
    this->windows[0]->x_goal = this->sett->x_goal;
    this->windows[0]->Qx_start = this->windows[0]->transformPoint(this->sett->x_start);
    this->windows[0]->Qx_goal = this->windows[0]->transformPoint(this->sett->x_goal);
}


//distruttore
Dialog::~Dialog(){

    //cancello windows
    for(auto w : this->windows){
        delete w;
    }

    //cancello planning data
    for(auto w : this->PDs1){
        delete w;
    }

    //cancello planning data
    for(auto w : this->PDs2){
        delete w;
    }

    //cancello planning problem
    delete this->PP;

    //pulisco settings
    delete sett;

    //delete panel
    delete this->panel;
    delete this;
}


//creo finestre di planning (si adattano al numero di threads)
void Dialog::createWindows(){

    //dimensione scena (fissa)
    qreal w = 550;
    qreal h = 350;

    t1 = qreal(this->PDs1.size()); //thread ABIT
    t2 = qreal(this->PDs2.size()); //thread RRT
    t = t1 + t2; //thread totali

    //setto dimensione variabile e tengo proporzione
    setMinimumSize(std::min(t, 3.0)*w + 50, std::min(3/t, 1.0)*h+100);
    setMaximumSize(std::min(t, 3.0)*w + 50, std::min(3/t, 1.0)*h+100);

    //layout principale che uso pen pannello centrale
    QGridLayout *rootLayout = new QGridLayout;

    //finestre ABIT
    for(int i=0; i<t1; i++){
        //aggiungo finestra
        this->windows.push_back(new GraphWidget(this, t, this->env));
        //creo label e setto font
        QLabel* lab = new QLabel("pABIT[" + QString::number(i) + "]" +
                                 " : " + QString::number(this->PDs1[i]->m) + " samples");
        QFont font = lab->font();
        font.setWeight(QFont::Bold);
        lab->setFont(font);
        lab->setIndent(200*pow(std::min(1.0, 3.0/t), 2));
        //aggiungo label e custom widget
        rootLayout->addWidget(lab, 0, i);
        rootLayout->addWidget(this->windows[i], 1, i);
    }
    //finestre RRT
    for(int i=0; i<t2; i++){
        //aggiungo finestra
        this->windows.push_back(new GraphWidget(this, t, this->env));
        //creo label e setto font
        QLabel* lab = new QLabel("pRRT[" + QString::number(i) + "]" +
                                 " : " + QString::number(this->PDs2[i]->m) + " gsr");
        QFont font = lab->font();
        font.setWeight(QFont::Bold);
        lab->setFont(font);
        lab->setIndent(200*pow(std::min(1.0, 3.0/t), 2));
        //aggiungo label e custom widget
        rootLayout->addWidget(lab, 0, i+t1);
        rootLayout->addWidget(this->windows[i+t1], 1, i+t1);
    }

    //widget centrale (base)
    this->panel = new QWidget;
    this->setWindowTitle("pABIT* (parallel ABIT*)");
    this->panel->setLayout(rootLayout);
    this->panel->show();
    this->panel->activateWindow();
    this->panel->raise();
}


//salvo file di testo
void Dialog::printData(std::vector<std::vector<float>> M){

    //apro file
    std::fstream out;
    out.open(this->sett->saveDir + "/DATA.txt", std::fstream::out);

    //ciclo e stampo
    unsigned long j, i;
    for(j=0; j<M.size(); j++){
        for(i=0; i<M[j].size()-1; i++){
            out << M[j][i] << '\t';
        }
        //nuova riga senza spazion finale/iniziale
        out << M[j][i] << std::endl;
    }
    out.close();
}


//----------- funzioni SLOT --------------------------
void Dialog::on_Run_Botton_clicked(){


    //-------------- Data storage --------------------
    //R = [thread1, time1, cost1, thread2, time2, cost2, ...], where type_ABIT = n + 10 and type_RRT = n + 20
    //M = {R_data_iter1, R_data_iter2, ...]
    std::vector<std::vector<float>> M(this->sett->iterations, std::vector<float>(3*this->t, 3*this->t));

    //aggiorno start e end ABIT
    for(unsigned long i = 0; i<t1; i++){
        PDs1[i]->x_start[0] = this->windows[0]->x_start.first;
        PDs1[i]->x_start[1] = this->windows[0]->x_start.second;
        PDs1[i]->x_goal[0] = this->windows[0]->x_goal.first;
        PDs1[i]->x_goal[1] = this->windows[0]->x_goal.second;
        //stampo start e goal per tutti
        //if(i>0 || !this->sett->draw){
        this->windows[i]->printStartEnd(this->windows[0]->Qx_start, this->windows[0]->Qx_goal);
        //}
    }

    //aggiorno start e end RRT
    for(unsigned long i = 0; i<t2; i++){
        PDs2[i]->x_start[0] = this->windows[0]->x_start.first;
        PDs2[i]->x_start[1] = this->windows[0]->x_start.second;
        PDs2[i]->x_goal[0] = this->windows[0]->x_goal.first;
        PDs2[i]->x_goal[1] = this->windows[0]->x_goal.second;
        //stampo start e goal per tutti
        //if(t1!=0 || !this->sett->draw)
        this->windows[i+t1]->printStartEnd(this->windows[0]->Qx_start, this->windows[0]->Qx_goal);
    }

    //eseguo planning a seconda che sia test o benchmark
    for(int k=0; k<this->sett->iterations; k++){

        //lancio il problema di planning
        this->PP = new PlanningProblem(this->windows, this->PDs1, this->PDs2);
        //richiedo soluzione
        Solution* result = this->PP->getPath();

        //scelgo se visualizzare --> soluzione troncata
        if(this->sett->visualization){
            if(result->Cbest < INF/2){
                this->ui->costEdit->setText("Cost: " + QString::number(round(result->Cbest*100)/100));
                this->ui->timeEdit->setText("Time: " + QString::number(round(result->time*1000)/1000) + " s");
                this->ui->winnerEdit->setText("Winner: " + QString::fromStdString(result->winner));
            }
            else {
                this->ui->costEdit->setText("Cost: INF");
                this->ui->timeEdit->setText("Time: _ s");
                this->ui->winnerEdit->setText("Winner: None");
            }
        }

        //soluzione completa --> aspetto tutti
        else {
            for(int j = 0; j<this->t; j++){
                for(int i=0; i<this->t; i++){
                    if(result->results_vec[i].first == 10 + j){
                        M[k][3*j] = result->results_vec[i].first;
                        M[k][3*j+1] = (result->results_vec[i].second).first;
                        M[k][3*j+2] = (result->results_vec[i].second).second;
                    }
                    else if(result->results_vec[i].first == 20 + j){
                        M[k][3*j+3*t1] = result->results_vec[i].first;
                        M[k][3*j+1+3*t1] = (result->results_vec[i].second).first;
                        M[k][3*j+2+3*t1] = (result->results_vec[i].second).second;
                    }
                }
            }
        }
    }

    //stampo risultati su file di testo
    if(!this->sett->visualization){
        this->printData(M);
        QMessageBox::information(this,"info","data saved");
        this->close();
    }
}


//do la possibilita di scegliere a schermo strt/end
void Dialog::on_Route_Button_clicked(){
    this->windows[0]->flagStartEnd = true;
    this->windows[0]->flagStart = true;
    this->windows[0]->flagEnd = true;
}
