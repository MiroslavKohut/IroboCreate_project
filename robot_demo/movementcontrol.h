#ifndef MOVEMENTCONTROL_H
#define MOVEMENTCONTROL_H

#include "irobotcreate.h"
#include "mapping.h"
#include "structures.h"


class MovementControl :public Mapping
{
public:
    MovementControl(float dt,iRobotCreate *robot);
    ~MovementControl();
    void moveToNewPose(float speed);
    void updatePose(float pose_change, float angle_change);
    void setPosReach(bool pos_reach_);
    void setPosAngle(bool ang_reach_);
    float degTorad(float data);
    float radTodeg(float data);

    float robRotateR(DWORD speed);
    float robRotateL(DWORD speed);
    float robMove(DWORD speed);
    void robStop();

    POSITION new_pose;
    POSITION irob_current_pose;



private:
    //variablese
    float dt;
    bool pos_reach;
    bool ang_reach;
    float target_angle;
    float target_dist;
    iRobotCreate *robot;

    POSITION irob_desired_pose;
    POSITION irob_start_pose;

    DWORD speed_up;
    DWORD speed_uppos;


    float dist_sum;

    //methods
    bool pidControlTranslation();
    bool pidControlRotation();
    float comuteAngle();
    float comuteTranslation();
    float comuteGoalAngle();
};


#endif // MOVEMENTCONTROL_H
