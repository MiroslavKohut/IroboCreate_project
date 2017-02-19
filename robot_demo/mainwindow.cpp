#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    irob_data = INFO_DATA();
    irob_pose = POSITION();

}

MainWindow::~MainWindow()
{
    delete ui;
}


int MainWindow::demoCallback(CreateSensors inputData,void *ioPointer)
{
    MainWindow *currentWindow=(MainWindow*)ioPointer;
  //  currentWindow->update();
    QString vystup = "Neplatny udaj.";
    QString vzdialenost= "Neplatny udaj.";
    QString uhol = "Neplatny udaj.";

    vystup = QString::number(inputData.Voltage);

    irob_data.prejdena_vzdialenost = irob_data.prejdena_vzdialenost + inputData.Distance;
    irob_data.uhol_otocenia = irob_data.uhol_otocenia + inputData.Angle;


    uhol = QString::number(irob_data.uhol_otocenia);
    vzdialenost = QString::number(irob_data.prejdena_vzdialenost);

    irob_pose.x = irob_pose.x + inputData.Distance*cos(inputData.Angle);
    irob_pose.y = irob_pose.y + inputData.Distance*sin(inputData.Andle);

    currentWindow->ui->label->setText(QString::number(inputData.Voltage));
    currentWindow->ui->distance->setText(vzdialenost);
    currentWindow->ui->angle->setText(uhol); 

    // currentWindow->ui->label-setText()
   //printf("nejde\n");
    //printf("data %i\n",inputData.BatteryCapacity);
}


void MainWindow::on_pushButton_clicked()
{
    robot.ConnectToPort("/dev/robot",this);
    connect( this, SIGNAL( showMB() ), this, SLOT( showMessageBox() ), Qt::BlockingQueuedConnection );
    robot.dataProcess(this,&demoCallback);
}

void MainWindow::on_pushButton_2_clicked()
{
    robot.move(-35,35);
    usleep(3000*1000);
    robot.move(50,50);
    usleep(3000*1000);
    robot.move(0,0);
}
