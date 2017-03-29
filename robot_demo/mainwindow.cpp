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
    static_map = new Mapping(false);
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


 double sx=500;
 double sy=30;

    if(paintEventStatus==14){

        static_map->clearMap();
        static_map->loadFile();

        std::vector<POINT> path;

        POINT start = {ui->start_x->text().toInt(),ui->start_y->text().toInt()};
        POINT end = {ui->stop_x->text().toInt(),ui->stop_y->text().toInt()};

        if(!static_map->findPath(path,start,end))
        {
            cout<< "NENASLO CESTU VYKRESLUJEM LEN MAPU" << endl;
        }


        QPainter painter(this);
        painter.setPen(Qt::white);
        painter.setBrush(Qt::white);
        int widtht=width()-50;
        int heightt=height();
        painter.drawRect(500,30,500,500); // zaciatok X=500 Y=30 Koniec X=740 Y=270 drawRect(x,y lavy horny,sirka vyska)
        painter.setPen(Qt::black);
        painter.setBrush(Qt::black);
        for(int x=0;x < MAP_WIDTH ;x++){
            for(int y=0;y < MAP_HIGHT;y++){
               if(static_map->map[x][y]== 1){
                   painter.setBrush(Qt::black);
                 painter.drawRect(500+x*5,30+y*5,5,5);
               }
               if(static_map->map[x][y]== 200){
                   painter.setBrush(Qt::red);
                 painter.drawRect(500+x*5,30+y*5,5,5);
               }
            }
        }
    }
    if(paintEventStatus==13)
    {
        //Mapping static_map(false);
        //static_map.loadFile();
        if (!robot){
            cout << "connect the robot" << endl;
        }
        else{

            QPainter painter(this);
            painter.setPen(Qt::white);
            painter.setBrush(Qt::white);
            int widtht=width()-50;
            int heightt=height();
            painter.drawRect(500,30,500,500); // zaciatok X=500 Y=30 Koniec X=740 Y=270 drawRect(x,y lavy horny,sirka vyska)
            painter.setPen(Qt::black);
            painter.setBrush(Qt::black);
            for(int x=0;x < MAP_WIDTH ;x++){
                for(int y=0;y < MAP_HIGHT;y++){
                   /*if(static_map.map[x][y]==1){
                       painter.setBrush(Qt::black);
                     painter.drawRect(500+x*10,30+y*10,10,10);
                   }*/
                   if(robot_movemet->map[x][y]==1){
                       painter.setBrush(Qt::black);
                     painter.drawRect(500+x*5,30+y*5,5,5);
                   }

                }
            }
            painter.setPen(Qt::red);
            painter.setBrush(Qt::red);
            int temp1=-(int)(robot_movemet->irob_current_pose.x/100)*10+500;
            int temp2=(int)(robot_movemet->irob_current_pose.y/100)*10+30;

            painter.drawRect(temp1,temp2,10,10);
        }
    }

    paintEventStatus = 0;
    QWidget::paintEvent(event);
}

void MainWindow::on_pushButton_6_clicked()
{

    paintEventStatus=13;
    update();
}

void MainWindow::on_pushButton_7_clicked()
{
    paintEventStatus=14;
    update();
}

void MainWindow::on_pushButton_8_clicked()
{
    robot_movemet->clearMap();
}
