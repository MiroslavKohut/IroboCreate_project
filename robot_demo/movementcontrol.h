#ifndef MOVEMENTCONTROL_H
#define MOVEMENTCONTROL_H

#include "irobotcreate.h"

struct POSITION{
    float x;
    float y;
    float angle;
};

class MovementControl
{
public:
    MovementControl(float dt,iRobotCreate *robot);
    ~MovementControl();
    void moveToNewPose(float speed);
    void updatePose(float pose_change, float angle_change);
    void setPosReach(bool pos_reach_);
    float degTorad(float data);

    float robRotateR(DWORD speed);
    float robRotateL(DWORD speed);
    float robMove(DWORD speed);
    void robStop();

    POSITION new_pose;




private:
    //variablese
    float dt;
    bool pos_reach;

    iRobotCreate *robot;

    POSITION irob_current_pose;
    POSITION irob_desired_pose;
    POSITION irob_start_pose;

    DWORD speed_up;
    float speed_sat;
    DWORD speed_uppos;

    u_int8_t movement_state;
    float dist_sum;

    //methods
    bool pidControlTranslation();
    bool pidControlRotation();
    float comuteAngle();
    float comuteTranslation();
};


#endif // MOVEMENTCONTROL_H
