#ifndef STRUCTURES_H
#define STRUCTURES_H

#define RAD_DEG M_PI/180
#define DEG_RAD 180/M_PI

typedef u_int8_t uint8_t ;
typedef struct POINT{
    float x;
    float y;
}POINT;

typedef struct POSITION{
    float x;
    float y;
    float angle;
}POSITION;

#endif // STRUCTURES_H
