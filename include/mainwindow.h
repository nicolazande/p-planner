#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "dialog.h" //includo header secondo form
#include <QMainWindow>
#include <iostream>
#include <string>
#include <QApplication>
#include <QTime>
#include <vector>
#include <map>
//custom classes
#include "graphwidget.h"
#include "pABITstar.h"
#include "solution.h"
#include "planningdata.h"
#include "planningproblem.h"
#include "env.h"
#include "ui_mainwindow.h"
#include <QString>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    //------------ funzioni -------------------------
    explicit MainWindow(QWidget *parent = nullptr); //costruttore
    ~MainWindow(); //distruttore
    std::vector<float> setupData(int type); //inizializzo vettore parametro thread

private slots:
    void on_Ok_Button_clicked(); //passo al planning
    void set_opt_threshold(int val); //setto threshold ottimalita
    void on_paddingSpinBox_valueChanged(const QString &arg1); //cambio padding
    void on_timeSpinBox_valueChanged(const QString &arg1); //cambio t_max
    void on_threadsCombo1_currentIndexChanged(int index); //threads ABIT
    void on_threadsCombo2_currentIndexChanged(int index); //threads RRT
    void on_Esc_Button_clicked(); //esc
    void on_testSwitch_clicked(); //test
    void on_benchmarkSwitch_clicked(); //benchmark
    void on_iterSpinBox_valueChanged(double arg1); //setto numero iterazioni
    void on_dirButton_clicked(); //setto directory dove salvo
    void on_envCombo_currentIndexChanged(int index); //cambio environment
    void on_startX_editingFinished(); //setto Xstart
    void on_startY_editingFinished(); //setto Ystart
    void on_endX_editingFinished(); //setto Xgoal
    void on_endY_editingFinished(); //setto Ygoal

private:
    //------------- variabili ------------------------
    Ui::MainWindow *ui; //puntatore a oggetto MainWindow
    Dialog *DialogUI; //dialog dove eseguo planning
    Env* env; //environment
    float ext[2][2]; //estremi, start e end
    float opt_threshold; //threshold
    float padding; //padding
    float max_time; //tempo massimo disponibile
    int threads1, threads2; //numero threads
    std::vector<PlanningData*> PDs1; //vettore plannings ABIT
    std::vector<PlanningData*> PDs2; //vettore plannings RRT
    Settings* sett;
};

#endif // MAINWINDOW_H
