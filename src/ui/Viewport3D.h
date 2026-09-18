#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include "cnc/Runtime.h"
class Viewport3D final : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
 cnc::Runtime* runtime_{};
public:
 explicit Viewport3D(cnc::Runtime* runtime,QWidget* parent=nullptr):QOpenGLWidget(parent),runtime_(runtime){}
protected:
 void initializeGL() override;
 void resizeGL(int w,int h) override;
 void paintGL() override;
};
