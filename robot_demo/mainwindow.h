#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include "irobotcreate.h"
#include "movementcontrol.h"

struct INFO_DATA
{
    float prejdena_vzdialenost;
    float uhol_otocenia;
};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
signals:
    void showMB();
public:

    void forMsgBox(){emit showMB();}

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    static int demoCallback(CreateSensors inputData,void *ioPointer);
    iRobotCreate *robot;

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();
    void showMessageBox()
            {
                QMessageBox Msgbox;
                Msgbox.setText("Save work and change Battery!");
                Msgbox.setIcon(QMessageBox::Critical);
                Msgbox.exec();
                exit(-1);
            }

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

private:
    Ui::MainWindow *ui;
    MovementControl *robot_movemet;

    float prejdena_vzdialenost;
    float uhol_otocenia;
    float napatie_bateriek;

    INFO_DATA irob_data;
};

#endif // MAINWINDOW_H
