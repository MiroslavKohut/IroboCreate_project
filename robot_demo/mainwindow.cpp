#include "mainwindow.h"
#include "ui_mainwindow.h"

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
    currentWindow->irob_data.uhol_otocenia =  currentWindow->irob_data.uhol_otocenia + inputData.Angle;


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
    robot.ConnectToPort("/dev/robot",this);
    connect( this, SIGNAL( showMB() ), this, SLOT( showMessageBox() ), Qt::BlockingQueuedConnection );
    robot.dataProcess(this,&demoCallback);

}

void MainWindow::on_pushButton_2_clicked()
{
   /* robot.move(-35,35);
    usleep(3000*1000);
    robot.move(50,50);
    usleep(3000*1000);
    robot.move(0,0);

*/

}
