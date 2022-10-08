#include "mainwindow.h"

//costruttore mainwindow --> eredita da QWindow e ui --> istanzia oggetti(IMPORTANTE)
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow){
    //istanzia tutti i widget contenuti --> IMPORTANTE
    ui->setupUi(this);
    this->setWindowTitle("Planning APP");
    //configurazioni iniziale e finale + environment setup
    int envIndex = 1;
    //setto configurazioni iniziali e finali per test
    this->ext[0][0] = 2;
    this->ext[0][1] = 20;
    this->ext[1][0] = 46;
    this->ext[1][1] = 21;
    this->max_time = 1;
    this->opt_threshold = 1.31f;
    this->padding = 0.25;//5 * this->ext[1][1]/100;
    this->threads1 = 1;
    this->threads2 = 0;
    this->env = new Env(envIndex);
    this->sett = new Settings(1,
                              true, qApp->applicationDirPath().toStdString(),
                              {this->ext[0][0], this->ext[0][1]},
                              {this->ext[1][0], this->ext[1][1]}); //settings base

    //connetto segnali
    QObject::connect(ui->optSlider,SIGNAL(valueChanged(int)), this, SLOT(set_opt_threshold(int)));
    QObject::connect(ui->Esc_Button, SIGNAL(clicked()), this, SLOT(on_Esc_Button_clicked()));

    //inizializzo form
    ui->optSlider->setValue(static_cast<int>((2-opt_threshold)*100));
    this->ui->optEdit->setText(QString::number(round(this->opt_threshold*100)/100));
    this->ui->paddingSpinBox->setValue(this->padding);
    this->ui->timeSpinBox->setValue(1);
    this->ui->threadsCombo1->setCurrentIndex(1);
    this->ui->threadsCombo2->setCurrentIndex(0);
    this->ui->startX->setText(QString::number(this->sett->x_start.first));
    this->ui->startY->setText(QString::number(this->sett->x_start.second));
    this->ui->endX->setText(QString::number(this->sett->x_goal.first));
    this->ui->endY->setText(QString::number(this->sett->x_goal.second));
    this->ui->envCombo->setCurrentIndex(envIndex);
}



//distruttore ui
MainWindow::~MainWindow(){
    delete this->sett; //cancello settings
    delete this->env; //cancello environment
    //cancello PDs1
    for(auto i : this->PDs1)
        delete i;
    //cancello PDs2
    for(auto i : this->PDs2)
        delete i;
    //cancello UI
    delete this->DialogUI;
    delete ui;
}


//setto threshold ottimalita
void MainWindow::set_opt_threshold(int val){
    this->opt_threshold = 1 + (1-static_cast<float>(val)/100);
    for(auto i : this->PDs1){
        i->threshold = this->opt_threshold;
    }
    for(auto i : this->PDs2){
        i->threshold = this->opt_threshold;
    }
    this->ui->optEdit->setText(QString::number(roundf(this->opt_threshold*1000)/1000));
}

//-------- funzioni per slot --------------------------
std::vector<float> MainWindow::setupData(int type){
    std::vector<float> threads_vec;
    //pABIT --> imposto samples_per_batch
    if(type == 0){
        this->PDs1 = {};
        switch (this->threads1) {
        case(1):
            threads_vec = {100};
            break;
        case(2):
            threads_vec = {50, 100};
            break;
        case(3):
            threads_vec = {50, 100, 150};
            break;
        case(4):
            threads_vec = {30, 60, 100, 150};
            break;
        case(5):
            threads_vec = {30, 50, 100, 100, 150};
            break;
        case(6):
            threads_vec = {20, 50, 100, 100, 120, 150};
            break;
        case(7):
            threads_vec = {20, 50, 80, 100, 100, 120, 150};
            break;
        case(8):
            threads_vec = {20, 50, 80, 100, 100, 100, 120, 150};
            break;
        default:
            threads_vec = {100};
        }
    }
    //pRRT --> imposto goal sample rate
    else if(type == 1){
        this->PDs2 = {};
        switch (this->threads2) {
        case(1):
            threads_vec = {0.05f};
            break;
        case(2):
            threads_vec = {0.05f, 0.10f};
            break;
        case(3):
            threads_vec = {0.05f, 0.10f, 0.15f};
            break;
        case(4):
            threads_vec = {0.05f, 0.10f, 0.15f, 0.20f};
            break;
        case(5):
            threads_vec = {0.05f, 0.10f, 0.10f, 0.15f, 0.20f};
            break;
        case(6):
            threads_vec = {0.02f, 0.05f, 0.10f, 0.10f, 0.15f, 0.20f};
            break;
        case(7):
            threads_vec = {0.02f, 0.05f, 0.10f, 0.10f, 0.15f, 0.20f, 0.25f};
            break;
        case(8):
            threads_vec = {0.02f, 0.05f, 0.10f, 0.10f, 0.10f, 0.15f, 0.20f, 0.25f};
            break;
        default:
            threads_vec = {0.05f};
        }
    }
    return threads_vec;
}



//passo al planning
void MainWindow::on_Ok_Button_clicked(){

    //creo setting thread parametri impostanti
    std::vector<float> threads_vec1 = this->setupData(0);
    std::vector<float> threads_vec2 = this->setupData(1);

    //creo planning data ABIT
    for(int i=0; i<this->threads1; i++){
        this->PDs1.push_back(new PlanningData(env, ext, i, 3.0f, this->padding, 0, this->sett->visualization,
                                              this->max_time, threads_vec1[i], this->opt_threshold));
    }

    //creo planning data ABIT
    for(int i=0; i<this->threads2; i++){
        this->PDs2.push_back(new PlanningData(env, ext, i, 3.0f, this->padding, 0, this->sett->visualization,
                                              this->max_time, threads_vec2[i], this->opt_threshold));
    }

    //lancio dialog
    DialogUI = new Dialog(this, this->env, this->PDs1, this->PDs2, this->sett);
    DialogUI->show();
}


//aggiusto padding con spin box
void MainWindow::on_paddingSpinBox_valueChanged(const QString &arg1){
    this->padding = arg1.toFloat() * this->ext[1][1]/100;
}


//tego conto dei thread che voglio usare ABIT
void MainWindow::on_threadsCombo1_currentIndexChanged(int index){
    this->threads1 = index;
}


//tego conto dei thread che voglio usare RRT
void MainWindow::on_threadsCombo2_currentIndexChanged(int index){
    this->threads2 = index;
}


//setto tempo massimo
void MainWindow::on_timeSpinBox_valueChanged(const QString &arg1){
    this->max_time = arg1.toFloat();
}


//bottone esc
void MainWindow::on_Esc_Button_clicked(){
    this->close();
}


//switch test
void MainWindow::on_testSwitch_clicked(){
    this->sett->visualization = true;
}


//switch benchmark
void MainWindow::on_benchmarkSwitch_clicked(){
    this->sett->visualization = false;
}


//setto numero iterazioni
void MainWindow::on_iterSpinBox_valueChanged(double arg1){
    this->sett->iterations = static_cast<int>(arg1);
}


//setto la directory dove salvare
void MainWindow::on_dirButton_clicked(){
    this->sett->saveDir = QFileDialog::getExistingDirectory().toStdString();
}


//cambio environment
void MainWindow::on_envCombo_currentIndexChanged(int index){
    delete this->env; //cancello environment vecchio
    this->env = new Env(index); //setto nuovo environment
}


//setto Xstart
void MainWindow::on_startX_editingFinished(){
    this->sett->x_start.first = this->ui->startX->text().toFloat();
    std::cout << this->ui->startX->text().toStdString() << std::endl;
}


//setto Ystart
void MainWindow::on_startY_editingFinished(){
    this->sett->x_start.second = this->ui->startY->text().toFloat();
    std::cout << this->ui->startY->text().toStdString() << std::endl;
}


//setto Xgoal
void MainWindow::on_endX_editingFinished(){
    this->sett->x_goal.first = this->ui->endX->text().toFloat();
    std::cout << this->ui->endX->text().toStdString() << std::endl;
}


//setto Ygoal
void MainWindow::on_endY_editingFinished(){
    this->sett->x_goal.second = this->ui->endY->text().toFloat();
    std::cout << this->ui->endY->text().toStdString() << std::endl;
}
