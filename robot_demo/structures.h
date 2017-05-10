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

typedef struct NAVIGATION_DATA{

    POSITION curent_position;
    float goal_angle;
    POSITION goal_point;
    bool bug_enabled;

}NAVIGATION_DATA;

typedef struct NAVIGATION_OUTPUT{

    float new_angle;
    bool data_ready;
    bool front_view_block;
    bool clear_path_to_goal;
    bool everything_blocked;

}NAVIGATION_OUTPUT;

typedef struct MAPPING_OUTPUT{

    std::vector<POINT> new_maping_pose;
    POINT start_point;
    POINT end_point;
    bool data_ready;

}MAPPING_OUTPUT;

#endif // STRUCTURES_H
