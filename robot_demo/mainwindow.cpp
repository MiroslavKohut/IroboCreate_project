#include "mainwindow.h"
#include "ui_mainwindow.h"

#define TIME_STAMP_S 0.007

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    irob_data = INFO_DATA();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete robot_movemet;
    delete robot;
}

int MainWindow::demoCallback(CreateSensors inputData,void *ioPointer)
{
    MainWindow *currentWindow=(MainWindow*)ioPointer;

  //  currentWindow->update();
    QString vystup = "Neplatny udaj.";
    QString vzdialenost= "Neplatny udaj.";
    QString uhol = "Neplatny udaj.";

    vystup = QString::number(inputData.Voltage);

    currentWindow->irob_data.prejdena_vzdialenost =  currentWindow->irob_data.prejdena_vzdialenost + inputData.Distance;
    currentWindow->irob_data.uhol_otocenia =  currentWindow->irob_data.uhol_otocenia -inputData.Angle;

    currentWindow->robot_movemet->updatePose(inputData.Distance,-inputData.Angle); //angle opacny z dovodu opacneho naprogramovania rotacii


    uhol = QString::number(currentWindow->irob_data.uhol_otocenia);
    vzdialenost = QString::number(currentWindow->irob_data.prejdena_vzdialenost);

    currentWindow->ui->label->setText(QString::number(inputData.Voltage));
    currentWindow->ui->distance->setText(vzdialenost);
    currentWindow->ui->angle->setText(uhol); 

    // currentWindow->ui->label-setText()
    //printf("nejde\n");
    //printf("data %i\n",inputData.BatteryCapacity);
}


void MainWindow::on_pushButton_clicked()
{
    robot= new iRobotCreate();
    robot->ConnectToPort("/dev/robot",this);
    connect( this, SIGNAL( showMB() ), this, SLOT( showMessageBox() ), Qt::BlockingQueuedConnection );
    robot->dataProcess(this,&demoCallback);

    robot_movemet = new MovementControl(TIME_STAMP_S,robot);
}

void MainWindow::on_pushButton_2_clicked()
{
    robot_movemet->robRotateR(100);
    usleep(2000*1000);
    robot_movemet->robMove(200);
    usleep(1000*1000);
    robot_movemet->robStop();
}
