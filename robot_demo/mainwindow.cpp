#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QPainter"

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
    //QString vzdialenost= "Neplatny udaj.";
    //QString uhol = "Neplatny udaj.";

    vystup = QString::number(inputData.Voltage);

    //currentWindow->irob_data.prejdena_vzdialenost =  currentWindow->irob_data.prejdena_vzdialenost + inputData.Distance;
    //currentWindow->irob_data.uhol_otocenia =  currentWindow->irob_data.uhol_otocenia -inputData.Angle;

    //uhol = QString::number(currentWindow->irob_data.uhol_otocenia);
    //vzdialenost = QString::number(currentWindow->irob_data.prejdena_vzdialenost);


    currentWindow->robot_movemet->updatePose(inputData.Distance,-inputData.Angle); //angle opacny z dovodu opacneho naprogramovania rotacii
    currentWindow->robot_movemet->moveToNewPose(50);


    //VYPIS
    vystup =QString::number(inputData.Voltage);
    currentWindow->ui->label->setText(vystup);
    //currentWindow->ui->distance->setText(vzdialenost);
    //currentWindow->ui->angle->setText(uhol);

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

void MainWindow::on_pushButton_2_clicked()
{
    if (robot != NULL){

        robot_movemet->new_pose.x = ui->lineEdit->text().toInt();
        robot_movemet->new_pose.y = ui->lineEdit_2->text().toInt();
        robot_movemet->new_pose.angle = 0;
        robot_movemet->setPosReach(false);
        robot_movemet->setPosAngle(false);
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
    }
}

void MainWindow::on_pushButton_4_clicked()
{
    if (!robot){
        cout << "connect the robot" << endl;
    }
    else{
        robot_movemet->startMapping();
    }
}

void MainWindow::on_pushButton_5_clicked()
{
    if (!robot){
        cout << "connect the robot" << endl;
    }
    else{
        robot_movemet->stopMapping();
    }
}

void MainWindow::paintEvent(QPaintEvent *event)
{

    if(paintEventStatus==13)
    {
        QPainter painter(this);
        painter.setPen(Qt::black);
        painter.setBrush(Qt::white);
        int widtht=width()-50;
        int heightt=height();
        painter.drawRect(500,30,240,240); // zaciatok X=500 Y=30 Koniec X=737 Y=267 drawRect(x,y lavy horny,sirka vyska)
        painter.setBrush(Qt::black);
        painter.drawRect(500,130,3,3);
         painter.drawRect(503,133,3,3);
        painter.drawRect(506,136,3,3);
        painter.drawRect(737,267,3,3);//koniec mapy



       /* Limeasure=lidar.getMeasurementFromFile();;
         printf("%i \n",Limeasure.timestamp);
        for(int i=0;i<Limeasure.numberOfScans;i++)
        {

            float dist = (1.5281*(Limeasure.Data[i].scanDistance)+1.9913);
            //QString vystup = "Neplatny udaj.";
            //if (dist >= 13.0) vystup = QString::number(dist);
            //double dist=rand()%(heightt/2);
            painter.drawPoint(50+widtht/2+cos(Limeasure.Data[i].scanAngle*3.14159/180.0)*dist,heightt/2+sin(Limeasure.Data[i].scanAngle*3.14159/180.0)*dist);
            //painter.drawRect(110,120,50,50);<
        }*/
        paintEventStatus = 0;


    }
    QWidget::paintEvent(event);
}

void MainWindow::on_pushButton_6_clicked()
{
    paintEventStatus=13;
    update();
}
