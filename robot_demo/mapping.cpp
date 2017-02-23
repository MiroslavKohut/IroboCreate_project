#include "mapping.h"
#include <math.h>
#include "rplidar.h"

Mapping::Mapping()
{
    lidar.connect("/dev/laser");
    lidar.enable();
    lidar.start();

    this->closest_distance = 0;
    this->closest_point = POINT();
}

Mapping::~Mapping()
{

}

void Mapping::getPoints(){

    LaserMeasurement measure=lidar.getMeasurement();
    float distance=0;
    float angle=0;
    POINT point;

    points.begin();
    for(int i=0; i<measure.numberOfScans;i++)
    {
        if (measure.Data[i].scanAngle <= 90)
            angle = abs(measure.Data[i].scanAngle - 90);
        else if(measure.Data[i].scanAngle > 90 && measure.Data[i].scanAngle <= 360){
            angle = 450 - measure.Data[i].scanAngle;
        }

        point.x = cos(angle)*measure.Data[i].scanDistance;
        point.y = sin(angle)*measure.Data[i].scanDistance;
        points.push_back(point);

        if (measure.Data[i].scanDistance > distance){
            distance = measure.Data[i].scanDistance;

            this->closest_point = point;
            this->closest_distance = distance;
        }

    }

}

bool Mapping::checkMovement(){

}

