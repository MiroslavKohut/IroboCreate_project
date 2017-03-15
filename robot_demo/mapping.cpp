#include "mapping.h"
#include <math.h>
#include "rplidar.h"

//50cm x 50cm start point // 500cm x 500cm area // 5cm x 5 cm square //

Mapping::Mapping()
{
    lidar.connect("/dev/laser");
    lidar.enable();
    lidar.start();
    mapping_run = false;
}

Mapping::~Mapping()
{
    pthread_join(mapping_thread,NULL);
}

bool Mapping::getMappingStatus(){

    pthread_mutex_lock (&mapping_status_lock);
    bool data = mapping_run;
    pthread_mutex_unlock (&mapping_status_lock);
    return data;
}

void Mapping::setMappingStatus(bool data){

    pthread_mutex_lock (&mapping_status_lock);
    mapping_run = data;
    pthread_mutex_unlock (&mapping_status_lock);
    return;
}

void Mapping::startMapping(){

    if(!getMappingStatus()){
        setMappingStatus(true);
        thread_id=pthread_create(&mapping_thread,NULL,&mappingThreadFun,(void *)this);
    }
    return;
}

void Mapping::stopMapping(){

    setMappingStatus(false);
    pthread_join(mapping_thread,NULL);

}

int Mapping::getPoints(){
    
    float angle=0;
    POINT point;

    while(getMappingStatus()){

        LaserMeasurement measure=lidar.getMeasurement();

        points.begin();
        for(int i=0; i<measure.numberOfScans;i++)
        {
            if (measure.Data[i].scanAngle <= 90)
                angle = abs(measure.Data[i].scanAngle - 90);
            else { //(measure.Data[i].scanAngle > 90 && measure.Data[i].scanAngle <= 360)
                angle = 450 - measure.Data[i].scanAngle;
            }

            point.x = cos(angle)*measure.Data[i].scanDistance;
            point.y = sin(angle)*measure.Data[i].scanDistance;
            points.push_back(point);
        }

    usleep(200000);
    }
    return 0;

}

bool Mapping::checkMovement(){

}

