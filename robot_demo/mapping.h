#ifndef MAPPING_H
#define MAPPING_H


#include "rplidar.h"
#include <iostream>
#include <vector>
#include <pthread.h>
#include "structures.h"
class Mapping
{
public:
    Mapping();
    ~Mapping();

private:
    /* methods */
    void setMappingStatus(bool data);
    bool getMappingStatus();
    /* variables */

    rplidar lidar;
    std::vector<POINT> points;
    POINT closest_point;
    float closest_distance;

    //thread variables
    bool mapping_run;
    pthread_t mapping_thread; // handle na vlakno
    int thread_id; //id vlakna
    int thread_out;

    pthread_mutex_t mapping_status_lock;
   /* pthread_mutex_lock (&mutexsum);
    dotstr.sum += mysum;
    pthread_mutex_unlock (&mutexsum);*/

public:

    int getPoints();
    bool checkMovement();
    void startMapping();
    void stopMapping();

    pthread_mutex_t current_pose_lock;

protected:
    POSITION irob_current_mapping_pose;

private:

    //--spustenie merania v novom vlakne (vycitavanie bezi v novom vlakne. treba ho stopnut ak chceme poslat request)
    static void *mappingThreadFun(void *param)
    {
        Mapping *mapping=(Mapping*)param;

        mapping->thread_out = mapping->getPoints();

        return &(mapping->thread_out);
    }

};

#endif // MAPPING_H

