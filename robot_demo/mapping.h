#ifndef MAPPING_H
#define MAPPING_H


#include "rplidar.h"
#include <iostream>
#include <vector>

struct POINT{
    float x;
    float y;
};

class Mapping
{
public:
    Mapping();
    ~Mapping();

private:
    /* variables */

    rplidar lidar;
    std::vector<POINT> points;
    POINT closest_point;
    float closest_distance;

protected:

    void getPoints();
    bool checkMovement();

};

#endif // MAPPING_H

