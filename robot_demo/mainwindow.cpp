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
    robot = NULL;
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

    uhol = QString::number(currentWindow->irob_data.uhol_otocenia);
    vzdialenost = QString::number(currentWindow->irob_data.prejdena_vzdialenost);


    currentWindow->robot_movemet->updatePose(inputData.Distance,-inputData.Angle); //angle opacny z dovodu opacneho naprogramovania rotacii
    currentWindow->robot_movemet->moveToNewPose(50);


    //VYPIS

    currentWindow->ui->label->setText(QString::number(inputData.Voltage));
    currentWindow->ui->distance->setText(vzdialenost);
    currentWindow->ui->angle->setText(uhol); 

}


void MainWindow::on_pushButton_clicked()
{
    if(robot == NULL){
        robot= new iRobotCreate();
        robot->ConnectToPort("/dev/robot",this);
        connect( this, SIGNAL( showMB() ), this, SLOT( showMessageBox() ), Qt::BlockingQueuedConnection );
        robot->dataProcess(this,&demoCallback);

        robot_movemet = new MovementControl(TIME_STAMP_S,robot);
    }
}

void MainWindow::on_pushButton_2_clicked(/*void *ioPointer*/)
{
    if (robot != NULL){

        robot_movemet->new_pose.x = ui->lineEdit->text().toInt();
        robot_movemet->new_pose.y = ui->lineEdit_2->text().toInt();
        /*robot_movemet->new_pose.angle = currentWindow->ui->lineEdit_3->text().toInt();
        robot_movemet->new_pose.x = 100;
        robot_movemet->new_pose.y = 100;*/
        robot_movemet->new_pose.angle = 0;
        robot_movemet->setPosReach(false);
        robot_movemet->setPosAngle(false);
        //TODO OTESTOVAT ATAN 2 a upravit  ratanie anglu
        /*new_pose.x = 0;
        new_pose.y = 100;
        new_pose.angle = 0;*/
        /*robot_movemet->robRotateR(100);
        usleep(2000*1000);
        robot_movemet->robMove(200);
        usleep(1000*1000);
        robot_movemet->robStop();
        */

    }
    else{

    cout << "connect the robot" << endl;
    }
}

void MainWindow::on_pushButton_3_clicked()
{

    if (!robot){
        cout << "connect the robot" << endl;
    }
    else{
    robot_movemet->setPosReach(true);
    robot_movemet->setPosAngle(true);
    robot_movemet->robStop();
    /*new_pose.x = 0;
    new_pose.y = 100;
    new_pose.angle = 0;*/
    /*robot_movemet->robRotateR(100);
    usleep(2000*1000);
    robot_movemet->robMove(200);
    usleep(1000*1000);
    robot_movemet->robStop();
    */
    }
}
