#include "Viewport3D.h"
#include <QPainter>
#include <QFont>
#include <cmath>
void Viewport3D::initializeGL(){initializeOpenGLFunctions();glEnable(GL_DEPTH_TEST);glClearColor(0.06f,0.07f,0.09f,1.0f);}
void Viewport3D::resizeGL(int w,int h){glViewport(0,0,w,h);}
void Viewport3D::paintGL(){
 glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
 QPainter p(this);p.setPen(Qt::white);p.setFont(QFont("Consolas",11));
 p.drawText(18,28,"CNC5AxisSim  |  3D MACHINE VIEW");
 if(!runtime_){p.end();return;}
 auto s=runtime_->state();
 p.drawText(18,52,QString("XYZAC  X:%1  Y:%2  Z:%3  A:%4  C:%5")
   .arg(s.X,0,'f',2).arg(s.Y,0,'f',2).arg(s.Z,0,'f',2).arg(s.A,0,'f',2).arg(s.C,0,'f',2));
 const int cx=width()/2, cy=height()/2;
 p.setPen(QPen(Qt::gray,2));p.drawLine(cx-260,cy,cx+260,cy);p.drawLine(cx,cy-180,cx,cy+180);
 p.setPen(QPen(Qt::green,4));p.drawEllipse(cx-5,cy-5,10,10);
 p.setPen(Qt::white);p.drawText(cx+12,cy-8,"TCP");
 p.end();
}
