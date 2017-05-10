#ifndef MAPPING_H
#define MAPPING_H


#include "rplidar.h"
#include <iostream>
#include <vector>
#include <pthread.h>
#include "structures.h"

#define MAP_WIDTH 100
#define MAP_HIGHT 100
#define DIV_CONST 50

class Mapping
{
public:
    Mapping(bool with_scan);
    ~Mapping();

private:
    /* variables */

    rplidar lidar;
    std::vector<POINT> points;
    std::vector<float> angles;
    POINT closest_point;
    float closest_distance;

    //thread variables
    bool mapping_run;
    bool navigation_run;
    pthread_t mapping_thread; // handle na vlakno
    int thread_id; //id vlakna
    int thread_out;
    NAVIGATION_DATA navigation_data;
    NAVIGATION_OUTPUT navigation_output;
    MAPPING_OUTPUT mapping_output;


    pthread_mutex_t mapping_status_lock;
    pthread_mutex_t navigation_status_lock;
    pthread_mutex_t navigation_data_lock;
    pthread_mutex_t mapping_output_lock;
    pthread_mutex_t mapppin_mutex;
   /* pthread_mutex_lock (&mutexsum);
    dotstr.sum += mysum;
    pthread_mutex_unlock (&mutexsum);*/

public:
       /* methods */

    void loadFile();
    int getPoints();
    bool checkMovement();

    void startMapping();
    void stopMapping();

    void startNavigation();
    void stopNavigation();

    void setMappingStatus(bool data);
    bool getMappingStatus();

    void setNavigationStatus(bool data);
    bool getNavigationStatus();
    MAPPING_OUTPUT getMappingOutput();
    void setMappingOutput(MAPPING_OUTPUT data);

    bool clearMap();
    void extract_number(std::string& line);
    bool findPath(std::vector<POINT> &cesta,POINT start, POINT end);

    std::vector< std::vector<uint8_t> > map;
    std::vector<POINT> new_maping_pose;

    pthread_mutex_t current_pose_lock;
    u_int8_t movement_state;

protected:

    POSITION irob_current_mapping_pose;

    void setNavigationOutput(NAVIGATION_OUTPUT data);
    NAVIGATION_OUTPUT getNavigationOutput();
    void setNavigationData(NAVIGATION_DATA data);
    NAVIGATION_DATA getNavigationData();

private:

    bool filterPath(std::vector<POINT> &path);
    void createDynamicMap(POINT bod);
    void mappingLoop();
    void navigationLoop();


    //--spustenie merania v novom vlakne (vycitavanie bezi v novom vlakne. treba ho stopnut ak chceme poslat request)
    static void *mappingThreadFun(void *param)
    {
        Mapping *mapping=(Mapping*)param;

        mapping->thread_out = mapping->getPoints();

        return &(mapping->thread_out);
    }

};

#endif // MAPPING_H

