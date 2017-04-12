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
    void paintEvent(QPaintEvent *event);
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

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_9_pressed();

    void on_pushButton_9_released();

    void on_pushButton_10_pressed();

    void on_pushButton_10_released();

    void on_pushButton_11_pressed();

    void on_pushButton_11_released();

private:
     int paintEventStatus;
    Ui::MainWindow *ui;
    MovementControl *robot_movemet;

    float prejdena_vzdialenost;
    float uhol_otocenia;
    float napatie_bateriek;
    Mapping *static_map;

    INFO_DATA irob_data;
};

#endif // MAINWINDOW_H
