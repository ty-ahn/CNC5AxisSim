#include "MainWindow.h"
#include "cnc/ProgramLoader.h"
#include <QPlainTextEdit>
#include <QLabel>
#include <QPushButton>
#include <QListWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
MainWindow::MainWindow(QWidget* p):QMainWindow(p),operator_(runtime_){
 setWindowTitle("CNC5AxisSim - Siemens 840D / 5-Axis");
 resize(1200,760);
 auto* root=new QWidget(this);auto* main=new QHBoxLayout(root);
 program_=new QPlainTextEdit; program_->setPlaceholderText("Paste Siemens 840D MPF program here...");
 main->addWidget(program_,2);
 auto* right=new QVBoxLayout; auto* pos=new QGridLayout;
 x_=new QLabel;y_=new QLabel;z_=new QLabel;a_=new QLabel;c_=new QLabel;feed_=new QLabel;rpm_=new QLabel;status_=new QLabel;
 const char* names[]={"X","Y","Z","A","C","F","S"}; QLabel* vals[]={x_,y_,z_,a_,c_,feed_,rpm_};
 for(int i=0;i<7;++i){pos->addWidget(new QLabel(names[i]),i,0);pos->addWidget(vals[i],i,1);}
 right->addLayout(pos);
 auto add=[&](const char* t,auto fn){auto* b=new QPushButton(t);connect(b,&QPushButton::clicked,this,fn);right->addWidget(b);};
 add("START",[this]{if(operator_.start())refresh();else refresh();});
 add("STEP",[this]{operator_.step();refresh();});
 add("HOLD",[this]{operator_.hold();refresh();});
 add("RESUME",[this]{operator_.resume();refresh();});
 add("STOP",[this]{operator_.stop();refresh();});
 add("RESET",[this]{operator_.reset();refresh();});
 auto* load=new QPushButton("LOAD MPF");connect(load,&QPushButton::clicked,this,[this]{QString fn=QFileDialog::getOpenFileName(this,"Open MPF",{}, "MPF (*.MPF *.mpf);;All Files (*)");if(fn.isEmpty())return;cnc::ProgramLoader l;std::string e;if(!l.load_file(fn.toStdString(),e)){QMessageBox::critical(this,"MPF",QString::fromStdString(e));return;}operator_.load(l.blocks());program_->setPlainText(QFileDialog().selectedFiles().isEmpty()?QString():QString());refresh();});right->addWidget(load);
 alarms_=new QListWidget;right->addWidget(alarms_,1);main->addLayout(right,1);setCentralWidget(root);loadDemo();
}
void MainWindow::loadDemo(){program_->setPlainText("N10 G90 G54 G0 X0 Y0 Z100\nN20 T1 D1 S8000 M3\nN30 G1 X100 Y50 Z20 A30 C45 F1000\nN40 G91 X5 C20\nN50 G90 G55 G0 X0 Y0 Z100\nN60 M5");cnc::ProgramLoader l;std::string e;l.load_text(program_->toPlainText().toStdString(),e);operator_.load(l.blocks());refresh();}
void MainWindow::refresh(){auto s=runtime_.state();x_->setText(QString::number(s.X,'f',3));y_->setText(QString::number(s.Y,'f',3));z_->setText(QString::number(s.Z,'f',3));a_->setText(QString::number(s.A,'f',3));c_->setText(QString::number(s.C,'f',3));feed_->setText(QString::number(runtime_.feed(),'f',1));rpm_->setText(QString::number(runtime_.rpm(),'f',0));status_->setText(QString::fromStdString(runtime_.alarm().empty()?"READY":runtime_.alarm()));if(alarms_){alarms_->clear();for(auto&e:runtime_.collisions().events())alarms_->addItem(QString("N%1  %2").arg(e.block_number).arg(QString::fromStdString(e.message)));}}
