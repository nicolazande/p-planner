#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H
#include <iostream>
#include <string>
#include <set>
#include <QGraphicsView>
#include <QList>
#include "env.h"
#include <QtWidgets>
#include <QMessageBox>

//------- graphwidget -----------------------------
class GraphWidget : public QGraphicsView
{
    Q_OBJECT

public:
    //--------- variabili --------------------------
    //flags
    bool flagStartEnd;
    bool flagStart;
    bool flagEnd;
    //initial and final configuration in different frames
    std::pair<float, float> x_start;
    std::pair<float, float> x_goal;
    std::pair<float, float> Qx_start;
    std::pair<float, float> Qx_goal;
    float t; //number threads

    //---------- funzioni --------------------------
    GraphWidget(QWidget *parent = 0, qreal t = 1, Env* env = nullptr); //costruttore
    ~GraphWidget(); //distruttore
    void buildPath(std::vector<std::pair<float, float>> points); //interpolo punti e plotto cunica
    void printStartEnd(std::pair<float, float> x1, std::pair<float, float> x2); //plotto start e goal per tutti
    std::pair<float, float> transformPoint(std::pair<float, float> p); //trasformo punto xy in window

private:
    //----------- variabili ------------------------
    QGraphicsScene* scene; //scena -> per modificarla
    Env* env; //envirionment -> lo uso spesso
    float w; //width ratio
    float h; //height ratio
    float err; //errore standard (incertezza)

    //----------- funzioni -------------------------
    void setupEnvironment(Env* env); //inizializzo environment
    bool eventFilter(QObject *object, QEvent *event); //eventfilter (uso per mouse click)
    void setStart(QPointF p); //setto configurazione iniziale + check obs
    void setEnd(QPointF p); //setto configurazione finale + check obs
    void vertex(float x, float y, QBrush); //plotto vertice
    void edge(std::pair<float, float> xp, std::pair<float, float> xc); //plotto edge
    QPair<QPointF, QPointF> controlPoints(QPointF const& p0, QPointF const& p1, QPointF const& p2, qreal t=0.25); //punti controllo cubica
    bool is_in_obs(float x, float y); //controllo se sono in ostacolo
};



#endif // GRAPHWIDGET_H
