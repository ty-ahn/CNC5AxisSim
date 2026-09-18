#include "MainWindow.h"
#include "Viewport3D.h"
#include "cnc/ProgramLoader.h"
#include "cnc/Verify.h"
#include <fstream>
#include <QPlainTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QTimer>
MainWindow::MainWindow(QWidget* p):QMainWindow(p),operator_(runtime_){
 setWindowTitle("CNC5AxisSim - Siemens 840D / 5-Axis");
 resize(1200,760);
 auto* root=new QWidget(this);auto* main=new QHBoxLayout(root);
 program_=new QPlainTextEdit; program_->setPlaceholderText("Paste Siemens 840D MPF program here...");
 auto* center=new QVBoxLayout; center->addWidget(new Viewport3D(&runtime_),2); center->addWidget(program_,1); main->addLayout(center,3);
 auto* timer=new QTimer(this); timer->setInterval(50); connect(timer,&QTimer::timeout,this,[this,timer]{ if(operator_.executor().state()==cnc::ExecutionState::Running){ if(!operator_.step()){timer->stop();} refresh(); } else timer->stop(); });
 auto* right=new QVBoxLayout; auto* pos=new QGridLayout;
 x_=new QLabel;y_=new QLabel;z_=new QLabel;a_=new QLabel;c_=new QLabel;feed_=new QLabel;rpm_=new QLabel;status_=new QLabel;
 const char* names[]={"X","Y","Z","A","C","F","S"}; QLabel* vals[]={x_,y_,z_,a_,c_,feed_,rpm_};
 for(int i=0;i<7;++i){pos->addWidget(new QLabel(names[i]),i,0);pos->addWidget(vals[i],i,1);}
 right->addLayout(pos); right->addWidget(status_);
 auto add=[&](const char* t,auto fn){auto* b=new QPushButton(t);connect(b,&QPushButton::clicked,this,fn);right->addWidget(b);};
 add("START",[this,timer]{ if(operator_.start()) { timer->start(); } refresh(); });
 add("STEP",[this]{operator_.step();refresh();centralWidget()->update();});
 add("HOLD",[this]{operator_.hold();refresh();});
 add("RESUME",[this]{operator_.resume();refresh();});
 add("STOP",[this,timer]{timer->stop();operator_.stop();refresh();});
 add("RESET",[this,timer]{timer->stop();operator_.reset();refresh();});
 add("VERIFY",[this]{cnc::ProgramLoader l;std::string e;if(!l.load_text(program_->toPlainText().toStdString(),e)){QMessageBox::critical(this,"VERIFY",QString::fromStdString(e));return;} auto r=cnc::VerifyEngine::run(l.blocks(),catalog_.subprograms()); QString msg=QString("VERIFY %1 | blocks %2/%3 | errors %4 | collisions %5").arg(r.passed?"PASS":"FAIL").arg((qulonglong)r.executed).arg((qulonglong)l.blocks().size()).arg((qulonglong)r.errors).arg((qulonglong)r.collisions); status_->setText(msg); if(!r.passed) QMessageBox::warning(this,"VERIFY",msg+"\n"+QString::fromStdString(r.first_error)); else QMessageBox::information(this,"VERIFY",msg);});
 auto* load=new QPushButton("LOAD MPF");connect(load,&QPushButton::clicked,this,[this]{QString fn=QFileDialog::getOpenFileName(this,"Open MPF",{}, "MPF (*.MPF *.mpf);;All Files (*)");if(fn.isEmpty())return;cnc::ProgramLoader l;std::string e;if(!l.load_file(fn.toStdString(),e)){QMessageBox::critical(this,"MPF",QString::fromStdString(e));return;}operator_.load(l.blocks()); catalog_.load_directory(QFileInfo(fn).absolutePath().toStdString(),e); for(const auto& [n,b]:catalog_.subprograms()) operator_.add_subprogram(n,b); std::ifstream in(fn.toStdString(),std::ios::binary);std::string text((std::istreambuf_iterator<char>(in)),std::istreambuf_iterator<char>());program_->setPlainText(QString::fromStdString(text)); refresh();});right->addWidget(load);
 alarms_=new QListWidget;right->addWidget(alarms_,1);main->addLayout(right,1);setCentralWidget(root);loadDemo();
}
void MainWindow::loadDemo(){program_->setPlainText("N10 G90 G54 G0 X0 Y0 Z100\nN20 T1 D1 S8000 M3\nN30 G1 X100 Y50 Z20 A30 C45 F1000\nN40 G91 X5 C20\nN50 G90 G55 G0 X0 Y0 Z100\nN60 M5");cnc::ProgramLoader l;std::string e;l.load_text(program_->toPlainText().toStdString(),e);operator_.load(l.blocks());refresh();}
void MainWindow::refresh(){auto s=runtime_.state();x_->setText(QString::number(s.X,'f',3));y_->setText(QString::number(s.Y,'f',3));z_->setText(QString::number(s.Z,'f',3));a_->setText(QString::number(s.A,'f',3));c_->setText(QString::number(s.C,'f',3));feed_->setText(QString::number(runtime_.feed(),'f',1));rpm_->setText(QString::number(runtime_.rpm(),'f',0));status_->setText(QString::fromStdString(runtime_.alarm().empty()?"READY":runtime_.alarm())); if(operator_.executor().state()==cnc::ExecutionState::Running) status_->setText(QString("RUN N%1 | P%2").arg((qulonglong)operator_.executor().current_block()).arg(operator_.executor().current_program()));if(alarms_){alarms_->clear();for(auto&e:runtime_.collisions().events())alarms_->addItem(QString("N%1  %2").arg(e.block_number).arg(QString::fromStdString(e.message)));}}
