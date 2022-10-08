#include "graphwidget.h"

//costruttore
GraphWidget::GraphWidget(QWidget *parent, qreal t, Env* env)
    : QGraphicsView(parent){

    //setup iniziale
    this->installEventFilter(this); //installo event filter
    this->flagStartEnd = false;
    this->flagStart = false;
    this->flagEnd = false;
    this->env = env; //inizializzo environment
    this->t = t; //numero threads
    this->err = 0.2; //errore standard

    //dimensioni singola finestra --> in funzione thread (3 = max)
    auto lx = env->obs_boundary(1) * 10 * std::min(1.0, qreal(3/t));
    auto ly = env->obs_boundary(3) * 10 * std::min(1.0, qreal(3/t));
    this->scene = new QGraphicsScene(this); //istanzio scena
    this->scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    this->scene->setSceneRect(0, 0, lx, ly); //dimensione scena (multipolo environment)
    setScene(this->scene); //imposto scena
    this->setWindowTitle("pABIT* (parallel ABIT*)");

    //ruoto origine per avere stesso orientamento
    scale(1, -1);

    //minima e massima dimensione totale
    setMinimumSize(1.08*lx, ly*1.2);
    setMaximumSize(1.08*lx, ly*1.2);

    //setup environment --> plotto tutto
    this->setupEnvironment(this->env);
}


//distruttore
GraphWidget::~GraphWidget(){
    //cancello anche scena
    delete this->scene;
    delete this;
}


//setup environment: plotto bordi e ostacoli
void GraphWidget::setupEnvironment(Env* env){

    QPen pen;
    //rapporti tra dimensione SS_immagine/SS_planning
    this->w = this->scene->width() / env->obs_boundary(1);
    this->h = this->scene->height() / env->obs_boundary(3);

    //dimensioni frame immagine corrispondenti
    auto length = env->obs_boundary(0, 1) * this->w;
    auto height = env->obs_boundary(0, 3) * this->h;

    //limiti state space immagine
    this->scene->addRect(QRect(-length/50, -length/50, 1.04*length, length/50),pen,QBrush(Qt::black));
    this->scene->addRect(QRect(-length/50, 0, length/50, height),pen,QBrush(Qt::black));
    this->scene->addRect(QRect(-length/50, height, 1.04*length, length/50),pen,QBrush(Qt::black));
    this->scene->addRect(QRect(length, 0, length/50, height),pen,QBrush(Qt::black));

    //ostacoli rettangolari
    auto r = env->obs_rectangle.rows();
    for(auto i=0; i<r; i++){
        auto xc = env->obs_rectangle(i,0) * this->w;
        auto yc = env->obs_rectangle(i, 1) *  this->h;
        auto length = env->obs_rectangle(i, 2) * this->w;
        auto height = env->obs_rectangle(i, 3) * this->h;
        QRect rec(xc, yc, length, height);
        this->scene->addRect(rec,pen,QBrush(Qt::green));
    }

    //ostacoli circolari
    r = env->obs_circle.rows();
    for(auto i=0; i<r; i++){
        auto xc = env->obs_circle(i,0) * this->w;
        auto yc = env->obs_circle(i, 1) *  this->h;
        auto radius = env->obs_circle(i, 2) * this->h;
        this->scene->addEllipse(xc, yc, radius, radius, pen, QBrush(Qt::green));
    }
}


//trasformazione inversa da xy --> schermo
std::pair<float, float> GraphWidget::transformPoint(std::pair<float, float> p){
    return {this->w * p.first, this->h * p.second};
}


//setto posizione start
void GraphWidget::setStart(QPointF p){

    QPen pen;
    auto sxy = std::min(1.0, qreal(3/this->t));
    float y = (this->scene->height()-p.y()+100*std::min(1.0, sqrt(sqrt(sxy))))/this->h;
    float x = (p.x()-55*sqrt(sxy))/this->w;

    //controllo se sono all'interno di un ostacolo
    if(this->is_in_obs(x, y)){
        QMessageBox::critical(this,"Error","Configuration in collision, retry");
    }
    else {
        this->x_start = {x, y}; //start per frame planning
        //setto start su scena --> frame locale
        this->Qx_start = {(p.x()-55*sqrt(sxy)), (this->scene->height()-p.y()+100*std::min(1.0, sqrt(sqrt(sxy))))};
        this->scene->addEllipse(this->Qx_start.first, this->Qx_start.second,
                                10, 10, pen, QBrush(Qt::red));
        this->flagStart = false; //spengo flag
    }
}


//setto posizione end
void GraphWidget::setEnd(QPointF p){

    QPen pen;
    auto sxy = std::min(1.0, qreal(3/this->t));
    float y = (this->scene->height()-p.y()+100*std::min(1.0, sqrt(sqrt(sxy))))/this->h;
    float x = (p.x()-55*sqrt(sxy))/this->w;

    //controllo se sono in un ostacolo
    if(this->is_in_obs(x, y)){
        QMessageBox::critical(this,"Error","Configuration in collision, retry");
    }
    else {
        this->x_goal = {x, y}; //end frame planning
        //setto end su scena --> frame locale
        this->Qx_goal = {(p.x()-55*sqrt(sxy)), (this->scene->height()-p.y()+100*std::min(1.0, sqrt(sqrt(sxy))))};
        //this->Qx_goal = {(p.x()-55*sqrt(sxy)), (this->scene->height()-p.y()+100)};
        this->scene->addEllipse(this->Qx_goal.first, this->Qx_goal.second,
                                10, 10, pen, QBrush(Qt::red));
        this->flagEnd = false;
    }
}


//controllo se dentro ostacolo
bool GraphWidget::is_in_obs(float x, float y){

    //controllo se sample interno a ostacoli cerchi
    auto r = this->env->obs_circle.rows();
    for (int i = 0; i < r; i++) {
        if(std::sqrt(pow(x - this->env->obs_circle(i, 0), 2) +
                     pow(y - this->env->obs_circle(i, 1), 2)) < this->env->obs_circle(i, 2) + this->err)
            return true;
    }

    //controllo se sample interno a ostacoli rettangolari
    r = this->env->obs_rectangle.rows();
    for (auto i = 0; i < r; i++) {
        if(x - this->env->obs_rectangle(i, 0) >=  -this->err &&
           x - this->env->obs_rectangle(i, 0) <= this->env->obs_rectangle(i, 2) + this->err &&
           y - this->env->obs_rectangle(i, 1) >= -this->err &&
           y - this->env->obs_rectangle(i, 1) <= this->env->obs_rectangle(i, 3) + this->err)
            return true;
    }
    return false;
}


//stampo start e goal
void GraphWidget::printStartEnd(std::pair<float, float> x1, std::pair<float, float> x2){
    QPen pen;
    this->scene->addEllipse(x1.first, x1.second,
                            10, 10, pen, QBrush(Qt::red));
    this->scene->addEllipse(x2.first, x2.second,
                            10, 10, pen, QBrush(Qt::red));
}


//event filter per prendere coordinate su schermo
bool GraphWidget::eventFilter(QObject *object, QEvent *event){

    //filtro solo click mouse
    if (event->type() == QEvent::MouseButtonPress){
       QMouseEvent* me = static_cast<QMouseEvent*>(event);
       //leggo posizione mouse
       QPointF windowPos = me->windowPos();

       //lo leggo solo se sto settando start o end
       if(this->flagStartEnd){
           if(this->flagStart){
               this->setStart(windowPos);
           }
           else if(this->flagEnd){
               this->setEnd(windowPos);
           }
        }
    return QObject::eventFilter(object, event);
    }
}


//aggiungo vertici
void GraphWidget::vertex(float x, float y, QBrush color){
    QPen pen;
    this->scene->addEllipse(x * this->w, y * this->h, 10, 10, pen, color);
}


//aggiungo edge
void GraphWidget::edge(std::pair<float, float> xp, std::pair<float, float> xc){
    QLine line(xp.first * this->w, xp.second * this->h, xc.first * this->w, xc.second * this->h);
    QPen pen = QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin);
    this->scene->addLine(line, pen);
}


//trovo coppia punti di controllo con interpolazione cubica
QPair<QPointF, QPointF> GraphWidget::controlPoints(QPointF const& p0, QPointF const& p1, QPointF const& p2, qreal t){
    QPair<QPointF, QPointF> pair;
    qreal d01 = std::sqrt( ( p1.x() - p0.x() ) * ( p1.x() - p0.x() ) + ( p1.y() - p0.y() ) * ( p1.y() - p0.y() ) );
    qreal d12 = std::sqrt( ( p2.x() - p1.x() ) * ( p2.x() - p1.x() ) + ( p2.y() - p1.y() ) * ( p2.y() - p1.y() ) );
    qreal fa = t * d01 / ( d01 + d12 );
    qreal fb = t * d12 / ( d01 + d12 );
    qreal c1x = p1.x() - fa * ( p2.x() - p0.x() );
    qreal c1y = p1.y() - fa * ( p2.y() - p0.y() );
    qreal c2x = p1.x() + fb * ( p2.x() - p0.x() );
    qreal c2y = p1.y() + fb * ( p2.y() - p0.y() );
    pair.first = QPointF( c1x, c1y );
    pair.second = QPointF( c2x, c2y );
    return pair;
}


//stampo path interpolato cubico
void GraphWidget::buildPath(std::vector<std::pair<float, float>> points){

    QPainterPath pth;
    QPair<QPointF, QPointF> pair = this->controlPoints(QPointF(points[0].first*this->w, points[0].second*this->h), \
            QPointF(points[1].first*this->w, points[1].second*this->h), QPointF(points[2].first*this->w, points[2].second*this->h));
    QPointF p0 = pair.second;

    //parto da fine
    pth.moveTo(QPointF(points[0].first*this->w, points[0].second*this->h));
    //pth.lineTo(p0); //--> posso anche partire con linea

    //interpolo cubico subito
    pth.cubicTo(pair.first, pair.second, QPointF(points[1].first*this->w, points[1].second*this->h));
    int i;
    for (i = 2; i < points.size() - 1; ++i){
        QPair<QPointF, QPointF> pair = this->controlPoints(QPointF(points[i-1].first*this->w, points[i-1].second*this->h),
                QPointF(points[i].first*this->w, points[i].second*this->h),
                QPointF(points[i+1].first*this->w, points[i+1].second*this->h));
        //interpolo cubico
        pth.cubicTo(p0, pair.first, QPointF(points[i].first*this->w, points[i].second*this->h));
        p0 = pair.second;
    }
    pth.cubicTo(p0, QPointF(points[i-1].first*this->w, points[i-1].second*this->h), QPointF(points[i].first*this->w, points[i].second*this->h));
    //plotto il path finale
    this->scene->addPath(pth);
}
