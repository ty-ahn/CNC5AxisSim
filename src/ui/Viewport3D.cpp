#include "Viewport3D.h"
#include <QPainter>
#include <QFont>
#include <cmath>
#include <algorithm>
#include <QMatrix4x4>
namespace {
cnc::Vec3 rv(cnc::Vec3 v, cnc::Vec3 axis, double deg){
 constexpr double pi=3.14159265358979323846; double n=std::sqrt(axis.x*axis.x+axis.y*axis.y+axis.z*axis.z);
 if(n<1e-12)return v; axis={axis.x/n,axis.y/n,axis.z/n}; double r=deg*pi/180.0,co=std::cos(r),si=std::sin(r);
 cnc::Vec3 cr{axis.y*v.z-axis.z*v.y,axis.z*v.x-axis.x*v.z,axis.x*v.y-axis.y*v.x}; double d=axis.x*v.x+axis.y*v.y+axis.z*v.z;
 return {v.x*co+cr.x*si+axis.x*d*(1-co),v.y*co+cr.y*si+axis.y*d*(1-co),v.z*co+cr.z*si+axis.z*d*(1-co)};
}
void rotatePivot(cnc::Vec3& p,const cnc::Vec3& pivot,const cnc::Vec3& axis,double deg){p.x-=pivot.x;p.y-=pivot.y;p.z-=pivot.z;p=rv(p,axis,deg);p.x+=pivot.x;p.y+=pivot.y;p.z+=pivot.z;}
cnc::Vec3 transformNode(cnc::Vec3 p,const cnc::MachineNode& n,const cnc::MachineState& state,const cnc::MachineKinematicConfig& k){
 std::string parent=n.parent;
 if(parent=="C"){ rotatePivot(p,k.pivot_c,k.c_axis,state.C); parent="A"; }
 if(parent=="A"){ rotatePivot(p,k.pivot_a,k.a_axis,state.A); parent="Z"; }
 if(parent=="Z") p.z+=state.Z;
 else if(parent=="Y") p.y+=state.Y;
 else if(parent=="X") p.x+=state.X;
 return p;
}
QPointF projectPoint(double X,double Y,double Z,int cx,int cy,float yaw,float pitch,float zoom){
 constexpr double pi=3.14159265358979323846;double yr=yaw*pi/180.0,pr=pitch*pi/180.0;
 double x=X*std::cos(yr)-Y*std::sin(yr),y=X*std::sin(yr)+Y*std::cos(yr),sy=y*std::cos(pr)-Z*std::sin(pr);
 return {cx+x*1.5*zoom,cy-sy*1.5*zoom};
}
cnc::Vec3 cross3(const cnc::Vec3&a,const cnc::Vec3&b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
double dot3(const cnc::Vec3&a,const cnc::Vec3&b){return a.x*b.x+a.y*b.y+a.z*b.z;}
cnc::Vec3 norm3(cnc::Vec3 v){double n=std::sqrt(dot3(v,v));return n<1e-12?cnc::Vec3{0,0,-1}:cnc::Vec3{v.x/n,v.y/n,v.z/n};}
cnc::Vec3 add3(const cnc::Vec3&a,const cnc::Vec3&b,double s=1.0){return {a.x+b.x*s,a.y+b.y*s,a.z+b.z*s};}
void addToolSurface(std::vector<float>& verts,const cnc::Vec3& tip,const cnc::Vec3& axis,double length,double radius,double noseRadius,double cr,double cg,double cb){
 if(length<=0||radius<=0)return; constexpr double pi=3.14159265358979323846; const int seg=32, rings=12; cnc::Vec3 w=std::fabs(axis.z)<0.9?cnc::Vec3{0,0,1}:cnc::Vec3{1,0,0}; cnc::Vec3 u=norm3(cross3(axis,w)),v=cross3(axis,u);
 auto add=[&](const cnc::Vec3&p){verts.insert(verts.end(),{float(p.x),float(p.y),float(p.z),cr,cg,cb});};
 auto point=[&](double z,double rr,double a){return add3(add3(add3(tip,axis,z),u,rr*std::cos(a)),v,rr*std::sin(a));};
 double nr=std::clamp(noseRadius,0.0,radius); int noseRings=nr>1e-9?std::max(2,int(rings*std::min(1.0,nr/length))):0;
 cnc::Vec3 prevTop{}; bool have=false;
 if(noseRings>0){for(int j=0;j<=noseRings;++j){double q=double(j)/noseRings;double z=nr*q;double rr=std::sqrt(std::max(0.0,nr*nr-(nr-z)*(nr-z))); if(j==0){for(int i=0;i<seg;++i){double a=2*pi*i/seg;point(z,rr,a);point(z,rr,2*pi*(i+1)/seg);point(z,rr,2*pi*(i+1)/seg);}} if(j>0){double z0=nr*double(j-1)/noseRings;double r0=std::sqrt(std::max(0.0,nr*nr-(nr-z0)*(nr-z0)));for(int i=0;i<seg;++i){double a0=2*pi*i/seg,a1=2*pi*(i+1)/seg;auto p00=add3(add3(add3(tip,axis,z0),u,r0*std::cos(a0)),v,r0*std::sin(a0));auto p01=add3(add3(add3(tip,axis,z0),u,r0*std::cos(a1)),v,r0*std::sin(a1));auto p10=add3(add3(add3(tip,axis,z),u,rr*std::cos(a0)),v,rr*std::sin(a0));auto p11=add3(add3(add3(tip,axis,z),u,rr*std::cos(a1)),v,rr*std::sin(a1));add(p00);add(p10);add(p11);add(p00);add(p11);add(p01);}}} double z0=nr; double r0=nr; for(int j=1;j<=rings;++j){double z=z0+(length-z0)*double(j)/rings;for(int i=0;i<seg;++i){double a0=2*pi*i/seg,a1=2*pi*(i+1)/seg;auto p00=add3(add3(add3(tip,axis,z0+(z-z0)*double(j-1)/rings),u,r0),v,0);auto p01=add3(add3(add3(tip,axis,z0+(z-z0)*double(j-1)/rings),u,r0*std::cos(a1)),v,r0*std::sin(a1));auto p10=add3(add3(add3(tip,axis,z),u,radius*std::cos(a0)),v,radius*std::sin(a0));auto p11=add3(add3(add3(tip,axis,z),u,radius*std::cos(a1)),v,radius*std::sin(a1));if(j==1){p00=add3(add3(tip,axis,z0),u,r0*std::cos(a0));} add(p00);add(p10);add(p11);add(p00);add(p11);add(p01);}}}
 else {for(int j=0;j<rings;++j){double z0=length*double(j)/rings,z1=length*double(j+1)/rings;for(int i=0;i<seg;++i){double a0=2*pi*i/seg,a1=2*pi*(i+1)/seg;auto A=add3(add3(add3(tip,axis,z0),u,radius*std::cos(a0)),v,radius*std::sin(a0));auto B=add3(add3(add3(tip,axis,z0),u,radius*std::cos(a1)),v,radius*std::sin(a1));auto C=add3(add3(add3(tip,axis,z1),u,radius*std::cos(a0)),v,radius*std::sin(a0));auto D=add3(add3(add3(tip,axis,z1),u,radius*std::cos(a1)),v,radius*std::sin(a1));add(A);add(C);add(D);add(A);add(D);add(B);}}}
}
void addCylinder(std::vector<float>& verts,const cnc::Vec3& p0,const cnc::Vec3& axis,double length,double radius,float cr,float cg,float cb){if(length<=0||radius<=0)return;constexpr double pi=3.14159265358979323846;const int seg=32;cnc::Vec3 a=norm3(axis),w=std::fabs(a.z)<0.9?cnc::Vec3{0,0,1}:cnc::Vec3{1,0,0},u=norm3(cross3(a,w)),v=cross3(a,u);auto add=[&](const cnc::Vec3&p){verts.insert(verts.end(),{float(p.x),float(p.y),float(p.z),cr,cg,cb});};for(int i=0;i<seg;++i){double a0=2*pi*i/seg,a1=2*pi*(i+1)/seg;auto A=add3(add3(p0,u,radius*std::cos(a0)),v,radius*std::sin(a0));auto B=add3(add3(p0,u,radius*std::cos(a1)),v,radius*std::sin(a1));auto C=add3(add3(p0,a,length),u,radius*std::cos(a0));C=add3(C,v,radius*std::sin(a0));auto D=add3(add3(p0,a,length),u,radius*std::cos(a1));D=add3(D,v,radius*std::sin(a1));add(A);add(C);add(D);add(A);add(D);add(B);}}
}
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
  const auto& nodes=runtime_->machine_render_nodes(); const auto& ms=runtime_->state(); const auto& kc=runtime_->kinematics().config(); for(size_t gi=0;gi<runtime_->machine_geometry().size();++gi){const auto& g=runtime_->machine_geometry()[gi]; if(!g.loaded())continue; const cnc::MachineNode* node=gi<nodes.size()?&nodes[gi]:nullptr; for(const auto& t:g.triangles()){auto a=node?transformNode(t.a,*node,ms,kc):t.a;auto b=node?transformNode(t.b,*node,ms,kc):t.b;auto d=node?transformNode(t.c,*node,ms,kc):t.c;addv(a,0.55f,0.58f,0.62f);addv(b,0.55f,0.58f,0.62f);addv(d,0.55f,0.58f,0.62f);}}
  const auto& sd=runtime_->stock().definition();
  int toolVertexStart=int(verts.size()/6);
  if(runtime_->tool()){
   cnc::ToolDefinition td{};
   if(runtime_->tool_table().get(runtime_->tool(),td)){
    const double L=std::max(0.0,td.length),R=std::max(0.0,td.diameter*0.5);
    auto axis=runtime_->kinematics().configured_axis_from_ac(ms.A,ms.C); auto tip=runtime_->kinematics().tcp_from_machine(ms,L);
    addToolSurface(verts,tip,axis,L,R,std::min(td.corner_radius,R),1.0f,0.78f,0.12f);
    auto top=add3(tip,axis,L); addCylinder(verts,top,axis,td.holder_length,std::max(0.0,td.holder_diameter*0.5),0.62f,0.64f,0.68f);
   }
  }

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
   int machineVertices=0;
   for(const auto& g:runtime_->machine_geometry()) if(g.loaded()) machineVertices+=int(g.triangles().size()*3);
   if(machineVertices>0) glDrawArrays(GL_TRIANGLES,0,machineVertices);
   if(toolVertexStart>machineVertices) glDrawArrays(GL_TRIANGLES,machineVertices,toolVertexStart-machineVertices);
   const int stockPointStart=toolVertexStart;
   if(stockPointStart<int(verts.size()/6)) glDrawArrays(GL_POINTS,stockPointStart,int(verts.size()/6)-stockPointStart);
   vbo_.release(); vao_.release(); shader_->release();
  }
 }
 QPainter p(this);p.setFont(QFont("Consolas",11));p.setPen(Qt::white);p.drawText(18,28,"CNC5AxisSim | 3D MACHINE VIEW");if(!runtime_){p.end();return;}auto s=runtime_->state();int cx=width()/2,cy=height()/2;auto project=[&](double X,double Y,double Z){return projectPoint(X,Y,Z,cx,cy,yaw_,pitch_,zoom_);};p.drawText(18,52,QString("XYZAC X:%1 Y:%2 Z:%3 A:%4 C:%5").arg(s.X,0,'f',2).arg(s.Y,0,'f',2).arg(s.Z,0,'f',2).arg(s.A,0,'f',2).arg(s.C,0,'f',2));p.drawText(18,72,QString("STOCK CELLS: %1 REMAINING: %2").arg((qulonglong)runtime_->stock().cell_count()).arg(runtime_->stock().remaining_volume(),0,'f',1));drawMachine(p,cx,cy);
 p.setPen(QPen(Qt::gray,1));p.drawLine(cx-300,cy,cx+300,cy);p.drawLine(cx,cy-220,cx,cy+220);const auto& m=runtime_->last_motion();if(m.size()>1){p.setPen(QPen(Qt::cyan,2));auto prev=project(m[0].X,m[0].Y,m[0].Z);for(size_t i=1;i<m.size();++i){auto cur=project(m[i].X,m[i].Y,m[i].Z);p.drawLine(prev,cur);prev=cur;}}double L=100;cnc::ToolDefinition td{};if(runtime_->tool()&&runtime_->tool_table().get(runtime_->tool(),td))L=td.length;auto axis=runtime_->kinematics().configured_axis_from_ac(s.A,s.C);auto tip=runtime_->kinematics().tcp_from_machine(s,L);auto toolTop=cnc::Vec3{tip.x+axis.x*L,tip.y+axis.y*L,tip.z+axis.z*L};auto a=project(tip.x,tip.y,tip.z),b=project(toolTop.x,toolTop.y,toolTop.z);p.setPen(QPen(Qt::yellow,4));p.drawLine(a,b);p.drawEllipse(a-QPointF(5,5),QPointF(5,5));p.drawText(a+QPointF(10,-10),"TOOL TIP");if(runtime_->tool()){double hl=td.holder_length;auto h0=cnc::Vec3{toolTop.x+axis.x*hl,toolTop.y+axis.y*hl,toolTop.z+axis.z*hl};p.setPen(QPen(Qt::gray,8));p.drawLine(project(toolTop.x,toolTop.y,toolTop.z),project(h0.x,h0.y,h0.z));p.setPen(QPen(Qt::yellow,3));p.drawLine(a,b);}for(const auto& ev:runtime_->collisions().events()){auto q=project(ev.event.position.x,ev.event.position.y,ev.event.position.z);p.setPen(QPen(Qt::red,4));p.drawEllipse(q-QPointF(9,9),QPointF(9,9));p.drawLine(q-QPointF(12,12),q+QPointF(12,12));p.drawLine(q-QPointF(12,-12),q+QPointF(12,-12));p.setPen(Qt::red);p.drawText(q+QPointF(14,4),QString("COLLISION N%1").arg(ev.block_number));}p.end();}
void Viewport3D::mousePressEvent(QMouseEvent*e){last_mouse_=e->pos();}
void Viewport3D::mouseMoveEvent(QMouseEvent*e){if(e->buttons()&Qt::LeftButton){auto d=e->pos()-last_mouse_;yaw_+=d.x()*0.5f;pitch_=std::clamp(pitch_+d.y()*0.5f,-89.0f,89.0f);last_mouse_=e->pos();update();}}
void Viewport3D::wheelEvent(QWheelEvent*e){zoom_=std::clamp(zoom_+e->angleDelta().y()/1200.0f,0.2f,5.0f);update();}
void Viewport3D::drawMachine(QPainter&p,int cx,int cy){if(!runtime_)return;auto s=runtime_->state();auto project=[&](double X,double Y,double Z){return projectPoint(X,Y,Z,cx,cy,yaw_,pitch_,zoom_);};p.setPen(QPen(Qt::gray,6));p.drawLine(project(-260,-180,0),project(260,-180,0));p.setPen(QPen(Qt::white,3));p.drawLine(project(-180,0,0),project(180,0,0));auto axis=runtime_->kinematics().configured_axis_from_ac(s.A,s.C);auto head=project(s.X,s.Y,s.Z);auto end=project(s.X-axis.x*70,s.Y-axis.y*70,s.Z-axis.z*70);p.setPen(QPen(Qt::yellow,5));p.drawLine(head,end);p.setPen(QPen(Qt::magenta,6));p.drawLine(head,project(s.X-axis.x*35,s.Y-axis.y*35,s.Z-axis.z*35));p.setPen(QPen(Qt::gray,10));p.drawLine(project(s.X-axis.x*35,s.Y-axis.y*35,s.Z-axis.z*35),end);p.setPen(QPen(Qt::cyan,3));p.drawEllipse(head-QPointF(8,8),QPointF(8,8));p.setPen(Qt::white);p.drawText(head+QPointF(12,20),QString("HEAD A%1 C%2").arg(s.A,0,'f',1).arg(s.C,0,'f',1));}
