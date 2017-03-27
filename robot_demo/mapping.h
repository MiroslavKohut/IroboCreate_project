#ifndef MAPPING_H
#define MAPPING_H


#include "rplidar.h"
#include <iostream>
#include <vector>
#include <pthread.h>
#include "structures.h"

#define MAP_WIDTH 50
#define MAP_HIGHT 50


class Mapping
{
public:
    Mapping(bool with_scan);
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
    pthread_mutex_t mapppin_mutex;
   /* pthread_mutex_lock (&mutexsum);
    dotstr.sum += mysum;
    pthread_mutex_unlock (&mutexsum);*/

public:
    void loadFile();
    int getPoints();
    bool checkMovement();
    void startMapping();
    void stopMapping();
    void extract_number(std::string& line);
    std::vector< std::vector<uint8_t> > map;

    pthread_mutex_t current_pose_lock;
    u_int8_t movement_state;

protected:

    POSITION irob_current_mapping_pose;

private:

    void createDynamicMap(POINT bod);
    void createStaticMap();
    //--spustenie merania v novom vlakne (vycitavanie bezi v novom vlakne. treba ho stopnut ak chceme poslat request)
    static void *mappingThreadFun(void *param)
    {
        Mapping *mapping=(Mapping*)param;

        mapping->thread_out = mapping->getPoints();

        return &(mapping->thread_out);
    }

};

#endif // MAPPING_H

