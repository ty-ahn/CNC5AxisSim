#include "Viewport3D.h"
#include <QPainter>
#include <QFont>
#include <cmath>
#include <QMatrix4x4>
#include <QPolygonF>
void Viewport3D::initializeGL(){initializeOpenGLFunctions();glEnable(GL_DEPTH_TEST);glClearColor(0.06f,0.07f,0.09f,1.0f);}
void Viewport3D::resizeGL(int w,int h){glViewport(0,0,w,h);}
void Viewport3D::paintGL(){
 glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
 update();
 QPainter p(this);p.setPen(Qt::white);p.setFont(QFont("Consolas",11));
 p.drawText(18,28,"CNC5AxisSim  |  3D MACHINE VIEW");
 if(!runtime_){p.end();return;}
 auto s=runtime_->state();
 p.drawText(18,72,QString("CUT VOXELS: %1 / %2").arg((qulonglong)cut).arg((qulonglong)runtime_->stock().cell_count()));
 p.drawText(18,52,QString("XYZAC  X:%1  Y:%2  Z:%3  A:%4  C:%5")
   .arg(s.X,0,'f',2).arg(s.Y,0,'f',2).arg(s.Z,0,'f',2).arg(s.A,0,'f',2).arg(s.C,0,'f',2));
 const int cx=width()/2, cy=height()/2;
 drawMachine(p,cx,cy);
 p.setPen(QPen(Qt::gray,2));p.drawLine(cx-260,cy,cx+260,cy);p.drawLine(cx,cy-180,cx,cy+180);
 p.setPen(QPen(Qt::darkGray,1));
 auto& motion=runtime_->last_motion();
 if(motion.size()>1){ p.setPen(QPen(Qt::cyan,2)); QPointF prev=project(motion[0].X,motion[0].Y,motion[0].Z); for(size_t i=1;i<motion.size();++i){ QPointF cur=project(motion[i].X,motion[i].Y,motion[i].Z); p.drawLine(prev,cur); prev=cur; } }
 QRectF stock(cx-220,cy-120,440,240); p.drawRect(stock);
 auto& sd=runtime_->stock().definition();
 if(runtime_->stock().cell_count()>0){
   const int nx=std::max(1,(int)std::ceil(sd.size_x/sd.resolution));
   const int ny=std::max(1,(int)std::ceil(sd.size_y/sd.resolution));
   const int nz=std::max(1,(int)std::ceil(sd.size_z/sd.resolution));
   const int stride=std::max(1,std::max({nx,ny,nz})/45);
   p.setPen(Qt::NoPen);
   for(int iz=0;iz<nz;iz+=stride) for(int iy=0;iy<ny;iy+=stride) for(int ix=0;ix<nx;ix+=stride)
     if(!runtime_->stock().is_removed(ix,iy,iz)){
       double X=sd.origin.x+(ix+0.5)*sd.resolution;
       double Y=sd.origin.y+(iy+0.5)*sd.resolution;
       double Z=sd.origin.z+(iz+0.5)*sd.resolution;
       QPointF q=project(X,Y,Z);
       p.drawRect(QRectF(q-QPointF(1,1),q+QPointF(1,1)));
     }
   p.setPen(Qt::white);
 }
 auto project=[&](double X,double Y,double Z){ double yr=yaw_*3.1415926535/180.0, pr=pitch_*3.1415926535/180.0; double x=X*std::cos(yr)-Y*std::sin(yr); double y=X*std::sin(yr)+Y*std::cos(yr); double sy=y*std::cos(pr)-Z*std::sin(pr); return QPointF(cx+x*1.5*zoom_,cy-sy*1.5*zoom_); };
 auto tcp=project(s.X,s.Y,s.Z);
 p.setPen(QPen(Qt::yellow,4)); p.drawLine(tcp,project(s.X,s.Y,s.Z+80));
 p.drawEllipse(tcp-QPointF(5,5),QPointF(5,5)); p.drawText(tcp+QPointF(10,-10),"TOOL/TCP");
 p.setPen(QPen(Qt::green,4));p.drawEllipse(cx-5,cy-5,10,10);
 p.setPen(Qt::white);p.drawText(cx+12,cy-8,"TCP");
 p.end();
}

void Viewport3D::mousePressEvent(QMouseEvent* e){last_mouse_=e->pos();}
void Viewport3D::mouseMoveEvent(QMouseEvent* e){if(e->buttons()&Qt::LeftButton){auto d=e->pos()-last_mouse_;yaw_+=d.x()*0.5f;pitch_=std::clamp(pitch_+d.y()*0.5f,-89.0f,89.0f);last_mouse_=e->pos();update();}}
void Viewport3D::wheelEvent(QWheelEvent* e){zoom_=std::clamp(zoom_+e->angleDelta().y()/1200.0f,0.2f,5.0f);update();}

void Viewport3D::drawMachine(QPainter& p,int cx,int cy){
 if(!runtime_) return;
 auto s=runtime_->state();
 auto project=[&](double X,double Y,double Z){double yr=yaw_*3.1415926535/180.0,pr=pitch_*3.1415926535/180.0;double x=X*std::cos(yr)-Y*std::sin(yr);double y=X*std::sin(yr)+Y*std::cos(yr);double sy=y*std::cos(pr)-Z*std::sin(pr);return QPointF(cx+x*1.5*zoom_,cy-sy*1.5*zoom_);};
 QPointF base=project(0,0,0), head=project(s.X,s.Y,s.Z);
 p.setPen(QPen(Qt::gray,6));p.drawLine(project(-260,-180,0),project(260,-180,0));
 p.setPen(QPen(Qt::white,3));p.drawLine(project(-180,0,0),project(180,0,0));
 double ar=s.A*3.1415926535/180.0, cr=s.C*3.1415926535/180.0;
 double ax=std::sin(ar)*std::cos(cr), ay=std::sin(ar)*std::sin(cr), az=-std::cos(ar);
 QPointF tip=project(s.X-ax*70,s.Y-ay*70,s.Z-az*70);
 p.setPen(QPen(Qt::yellow,5));p.drawLine(head,tip);
 p.setPen(QPen(Qt::magenta,6));p.drawLine(head,project(s.X-ax*35,s.Y-ay*35,s.Z-az*35));
 double hx=s.X-ax*35,hy=s.Y-ay*35,hz=s.Z-az*35;
 p.setPen(QPen(Qt::gray,10));p.drawLine(project(hx,hy,hz),project(hx+ax*70,hy+ay*70,hz+az*70));
 p.setPen(QPen(Qt::yellow,5));
 for(int i=0;i<6;++i){double t=i/5.0; QPointF q=project(s.X-ax*(70*t),s.Y-ay*(70*t),s.Z-az*(70*t));p.drawEllipse(q-QPointF(3,3),QPointF(3,3));}
 p.drawText(tip+QPointF(8,-8),QString("TOOL A%1 C%2").arg(s.A,0,'f',1).arg(s.C,0,'f',1));
 p.setPen(QPen(Qt::cyan,3));p.drawEllipse(head-QPointF(8,8),QPointF(8,8));
 p.drawText(head+QPointF(12,20),QString("HEAD A%1 C%2").arg(s.A,0,'f',1).arg(s.C,0,'f',1));
}
