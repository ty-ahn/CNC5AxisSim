#pragma once
#include "cnc/OperatorController.h"
#include "cnc/ProgramCatalog.h"
#include <QMainWindow>
class QPlainTextEdit; class QLabel; class QPushButton; class QListWidget;
class MainWindow final : public QMainWindow {
 Q_OBJECT
 cnc::Runtime runtime_; cnc::OperatorController operator_; cnc::ProgramCatalog catalog_;
 QPlainTextEdit* program_{};
 QLabel *x_{},*y_{},*z_{},*a_{},*c_{},*feed_{},*rpm_{},*status_{};
 QListWidget* alarms_{};
 void refresh();
 void loadDemo();
public:
 explicit MainWindow(QWidget* parent=nullptr);
};
