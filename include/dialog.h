#ifndef DIALOG_H
#define DIALOG_H
#include <fstream>
#include <QHBoxLayout>
#include <iostream>
#include <QGraphicsView>
#include <QMainWindow>
#include <QtWidgets>
#include "graphwidget.h"
#include "env.h"
#include <QDialog>
#include "planningdata.h"
#include "planningproblem.h"
#include "ui_dialog.h"


class PlanningData;


namespace Ui {
class Dialog;
}

class Dialog : public QDialog{
    Q_OBJECT

public:
    //------ variabili --------------
    QGraphicsScene* scene; //scena generale
    std::vector<GraphWidget*> windows; //vettore di wiget pointers, uno per thread
    QWidget *panel; //widget centrale che contiene altri

    //------- funzioni --------------
    explicit Dialog(QWidget *parent = nullptr, Env* env = nullptr,
                    std::vector<PlanningData*> PDs1 = std::vector<PlanningData*>(),
                    std::vector<PlanningData*> PDs2 = std::vector<PlanningData*>(),
                    Settings* sett = nullptr); //costruttore
    ~Dialog(); //distruttore
    void printData(std::vector<std::vector<float>> M);

private slots:
    void on_Run_Botton_clicked(); //faccio iniziare il planning
    void on_Route_Button_clicked(); //setto start e end

private:
    //------- variabili -------------
    Ui::Dialog *ui;
    QPushButton* start;
    PlanningProblem*  PP; //planning problem nuovo
    std::vector<PlanningData*> PDs1; //vettore plannings ABIT
    std::vector<PlanningData*> PDs2; //vettore plannings RRT
    Settings* sett;
    Env* env;
    QHBoxLayout *rootLayout;
    qreal t1, t2, t;

    //------- funzioni ---------------
    void createWindows(); //istanzia singole finestre planning
};

#endif // DIALOG_H
