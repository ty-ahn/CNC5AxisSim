#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <vector>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include "cnc/Runtime.h"
class Viewport3D final : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
 cnc::Runtime* runtime_{};
 QPoint last_mouse_{}; float yaw_=35.0f,pitch_=25.0f,zoom_=1.0f; QOpenGLShaderProgram* shader_{}; QOpenGLBuffer vbo_{QOpenGLBuffer::VertexBuffer}; QOpenGLVertexArrayObject vao_;
 void drawMachine(QPainter& p,int cx,int cy); 
public:
 explicit Viewport3D(cnc::Runtime* runtime,QWidget* parent=nullptr):QOpenGLWidget(parent),runtime_(runtime){}
protected:
 void mousePressEvent(QMouseEvent*) override;
 void mouseMoveEvent(QMouseEvent*) override;
 void wheelEvent(QWheelEvent*) override;
 void initializeGL() override;
 void resizeGL(int w,int h) override;
 void paintGL() override;
};
