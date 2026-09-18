#include "Viewport3D.h"
#include <QPainter>
#include <QFont>
#include <cmath>
#include <algorithm>
#include <QMatrix4x4>
namespace { QPointF projectPoint(double X,double Y,double Z,int cx,int cy,float yaw,float pitch,float zoom){constexpr double pi=3.14159265358979323846;double yr=yaw*pi/180.0,pr=pitch*pi/180.0;double x=X*std::cos(yr)-Y*std::sin(yr),y=X*std::sin(yr)+Y*std::cos(yr),sy=y*std::cos(pr)-Z*std::sin(pr);return {cx+x*1.5*zoom,cy-sy*1.5*zoom};} }
void Viewport3D::initializeGL(){
 initializeOpenGLFunctions(); glEnable(GL_DEPTH_TEST); glEnable(GL_PROGRAM_POINT_SIZE); glClearColor(0.06f,0.07f,0.09f,1.0f);
 shader_=new QOpenGLShaderProgram(this);
 shader_->addShaderFromSourceCode(QOpenGLShader::Vertex,"#version 330 core\nin vec3 position; in vec3 vertexColor; uniform mat4 mvp; out vec3 color; void main(){gl_Position=mvp*vec4(position,1.0); color=vertexColor; gl_PointSize=3.0;}");
 shader_->addShaderFromSourceCode(QOpenGLShader::Fragment,"#version 330 core\nin vec3 color; out vec4 frag; void main(){frag=vec4(color,1.0);}");
 if(!shader_->link()) delete shader_, shader_=nullptr;
 vao_.create(); vbo_.create();
}
void Viewport3D::resizeGL(int w,int h){glViewport(0,0,w,h);}
void Viewport3D::paintGL(){
 glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
 if(shader_ && shader_->isLinked() && runtime_){
  QMatrix4x4 proj,view; proj.perspective(45.0f,float(width())/std::max(1,height()),0.1f,5000.0f);
  view.translate(0,0,-700.0f/zoom_); view.rotate(pitch_,1,0,0); view.rotate(yaw_,0,0,1);
  QMatrix4x4 mvp=proj*view;
  std::vector<float> verts; verts.reserve(30000);
  auto addv=[&](const cnc::Vec3& p,float r,float g,float b){verts.insert(verts.end(),{float(p.x),float(p.y),float(p.z),r,g,b});};
  for(const auto& g:runtime_->machine_geometry()) if(g.loaded()) for(const auto& t:g.triangles()){
   addv(t.a,0.55f,0.58f,0.62f); addv(t.b,0.55f,0.58f,0.62f); addv(t.c,0.55f,0.58f,0.62f);
  }
  const auto& sd=runtime_->stock().definition();
  if(runtime_->stock().cell_count()>0){
   const int nx=std::max(1,int(std::ceil(sd.size_x/std::max(0.001,sd.resolution))));
   const int ny=std::max(1,int(std::ceil(sd.size_y/std::max(0.001,sd.resolution))));
   const int nz=std::max(1,int(std::ceil(sd.size_z/std::max(0.001,sd.resolution))));
   const double rx=sd.size_x/nx,ry=sd.size_y/ny,rz=sd.size_z/nz;
   const size_t max_points=120000;
   size_t emitted=0;
   for(int z=0;z<nz && emitted<max_points;++z) for(int y=0;y<ny && emitted<max_points;++y) for(int x=0;x<nx && emitted<max_points;++x)
    if(!runtime_->stock().is_removed(x,y,z)){addv({sd.origin.x+(x+0.5)*rx,sd.origin.y+(y+0.5)*ry,sd.origin.z+(z+0.5)*rz},0.20f,0.65f,0.95f);++emitted;}
   glPointSize(3.0f);
  }
  if(!verts.empty()){
   shader_->bind(); shader_->setUniformValue("mvp",mvp); vao_.bind(); vbo_.bind();
   vbo_.allocate(verts.data(),int(verts.size()*sizeof(float)));
   shader_->enableAttributeArray("position"); shader_->setAttributeBuffer("position",GL_FLOAT,0,3,6*sizeof(float));
   shader_->enableAttributeArray("vertexColor"); shader_->setAttributeBuffer("vertexColor",GL_FLOAT,3*sizeof(float),3,6*sizeof(float));
   int machineFloats=0;
   for(const auto& g:runtime_->machine_geometry()) if(g.loaded()) machineFloats+=int(g.triangles().size()*18);
   if(machineFloats>0) glDrawArrays(GL_TRIANGLES,0,machineFloats/6);
   const int stockStart=machineFloats/6;
   if(stockStart<int(verts.size()/6)) glDrawArrays(GL_POINTS,stockStart,int(verts.size()/6)-stockStart);
   vbo_.release(); vao_.release(); shader_->release();
  }
 }
 QPainter p(this);p.setFont(QFont("Consolas",11));p.setPen(Qt::white);p.drawText(18,28,"CNC5AxisSim | 3D MACHINE VIEW");if(!runtime_){p.end();return;}auto s=runtime_->state();int cx=width()/2,cy=height()/2;auto project=[&](double X,double Y,double Z){return projectPoint(X,Y,Z,cx,cy,yaw_,pitch_,zoom_);};p.drawText(18,52,QString("XYZAC X:%1 Y:%2 Z:%3 A:%4 C:%5").arg(s.X,0,'f',2).arg(s.Y,0,'f',2).arg(s.Z,0,'f',2).arg(s.A,0,'f',2).arg(s.C,0,'f',2));p.drawText(18,72,QString("STOCK CELLS: %1 REMAINING: %2").arg((qulonglong)runtime_->stock().cell_count()).arg(runtime_->stock().remaining_volume(),0,'f',1));drawMachine(p,cx,cy);
 p.setPen(QPen(Qt::gray,1));p.drawLine(cx-300,cy,cx+300,cy);p.drawLine(cx,cy-220,cx,cy+220);const auto& m=runtime_->last_motion();if(m.size()>1){p.setPen(QPen(Qt::cyan,2));auto prev=project(m[0].X,m[0].Y,m[0].Z);for(size_t i=1;i<m.size();++i){auto cur=project(m[i].X,m[i].Y,m[i].Z);p.drawLine(prev,cur);prev=cur;}}double L=100;cnc::ToolDefinition td{};if(runtime_->tool()&&runtime_->tool_table().get(runtime_->tool(),td))L=td.length;auto axis=runtime_->kinematics().configured_axis_from_ac(s.A,s.C);auto tcp=runtime_->kinematics().tcp_from_machine(s,L);auto a=project(tcp.x,tcp.y,tcp.z),b=project(tcp.x-axis.x*60,tcp.y-axis.y*60,tcp.z-axis.z*60);p.setPen(QPen(Qt::yellow,4));p.drawLine(a,b);p.drawEllipse(a-QPointF(5,5),QPointF(5,5));p.drawText(a+QPointF(10,-10),"TOOL/TCP");for(const auto& ev:runtime_->collisions().events()){auto q=project(ev.event.position.x,ev.event.position.y,ev.event.position.z);p.setPen(QPen(Qt::red,4));p.drawEllipse(q-QPointF(9,9),QPointF(9,9));p.drawLine(q-QPointF(12,12),q+QPointF(12,12));p.drawLine(q-QPointF(12,-12),q+QPointF(12,-12));p.setPen(Qt::red);p.drawText(q+QPointF(14,4),QString("COLLISION N%1").arg(ev.block_number));}p.end();}
void Viewport3D::mousePressEvent(QMouseEvent*e){last_mouse_=e->pos();}
void Viewport3D::mouseMoveEvent(QMouseEvent*e){if(e->buttons()&Qt::LeftButton){auto d=e->pos()-last_mouse_;yaw_+=d.x()*0.5f;pitch_=std::clamp(pitch_+d.y()*0.5f,-89.0f,89.0f);last_mouse_=e->pos();update();}}
void Viewport3D::wheelEvent(QWheelEvent*e){zoom_=std::clamp(zoom_+e->angleDelta().y()/1200.0f,0.2f,5.0f);update();}
void Viewport3D::drawMachine(QPainter&p,int cx,int cy){if(!runtime_)return;auto s=runtime_->state();auto project=[&](double X,double Y,double Z){return projectPoint(X,Y,Z,cx,cy,yaw_,pitch_,zoom_);};p.setPen(QPen(Qt::gray,6));p.drawLine(project(-260,-180,0),project(260,-180,0));p.setPen(QPen(Qt::white,3));p.drawLine(project(-180,0,0),project(180,0,0));auto axis=runtime_->kinematics().configured_axis_from_ac(s.A,s.C);auto head=project(s.X,s.Y,s.Z);auto end=project(s.X-axis.x*70,s.Y-axis.y*70,s.Z-axis.z*70);p.setPen(QPen(Qt::yellow,5));p.drawLine(head,end);p.setPen(QPen(Qt::magenta,6));p.drawLine(head,project(s.X-axis.x*35,s.Y-axis.y*35,s.Z-axis.z*35));p.setPen(QPen(Qt::gray,10));p.drawLine(project(s.X-axis.x*35,s.Y-axis.y*35,s.Z-axis.z*35),end);p.setPen(QPen(Qt::cyan,3));p.drawEllipse(head-QPointF(8,8),QPointF(8,8));p.setPen(Qt::white);p.drawText(head+QPointF(12,20),QString("HEAD A%1 C%2").arg(s.A,0,'f',1).arg(s.C,0,'f',1));}
