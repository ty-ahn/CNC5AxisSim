#include "Viewport3D.h"
#include <QPainter>
#include <QFont>
#include <cmath>
#include <QMatrix4x4>
void Viewport3D::initializeGL(){initializeOpenGLFunctions();glEnable(GL_DEPTH_TEST);glClearColor(0.06f,0.07f,0.09f,1.0f);}
void Viewport3D::resizeGL(int w,int h){glViewport(0,0,w,h);}
void Viewport3D::paintGL(){
 glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
 update();
 QPainter p(this);p.setPen(Qt::white);p.setFont(QFont("Consolas",11));
 p.drawText(18,28,"CNC5AxisSim  |  3D MACHINE VIEW");
 if(!runtime_){p.end();return;}
 auto s=runtime_->state();
 p.drawText(18,52,QString("XYZAC  X:%1  Y:%2  Z:%3  A:%4  C:%5")
   .arg(s.X,0,'f',2).arg(s.Y,0,'f',2).arg(s.Z,0,'f',2).arg(s.A,0,'f',2).arg(s.C,0,'f',2));
 const int cx=width()/2, cy=height()/2;
 p.setPen(QPen(Qt::gray,2));p.drawLine(cx-260,cy,cx+260,cy);p.drawLine(cx,cy-180,cx,cy+180);
 p.setPen(QPen(Qt::darkGray,1));
 auto& motion=runtime_->last_motion();
 if(motion.size()>1){ p.setPen(QPen(Qt::cyan,2)); QPointF prev=project(motion[0].X,motion[0].Y,motion[0].Z); for(size_t i=1;i<motion.size();++i){ QPointF cur=project(motion[i].X,motion[i].Y,motion[i].Z); p.drawLine(prev,cur); prev=cur; } }
 QRectF stock(cx-220,cy-120,440,240); p.drawRect(stock);
 auto project=[&](double X,double Y,double Z){double sx=X*1.5; double sy=-Y*1.5-Z*0.55; return QPointF(cx+sx,cy+sy);};
 auto tcp=project(s.X,s.Y,s.Z);
 p.setPen(QPen(Qt::yellow,4)); p.drawLine(tcp,project(s.X,s.Y,s.Z+80));
 p.drawEllipse(tcp-QPointF(5,5),QPointF(5,5)); p.drawText(tcp+QPointF(10,-10),"TOOL/TCP");
 p.setPen(QPen(Qt::green,4));p.drawEllipse(cx-5,cy-5,10,10);
 p.setPen(Qt::white);p.drawText(cx+12,cy-8,"TCP");
 p.end();
}
