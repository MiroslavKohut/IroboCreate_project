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
    MovementControl(float dt, iRobotCreate& robot);
    ~MovementControl();
    void moveToNewPose(float speed, POSITION pose);
    void updatePose(float pose_change, float angle_change);

private:
    //variablese
    float dt;

    iRobotCreate robot;
    POSITION irob_current_pose;
    POSITION irob_desired_pose;

    //methods
    bool pidControlTranslation();
    bool pidControlRotation();
    float comuteAngle();
    float comuteTranslation();
};


#endif // MOVEMENTCONTROL_H
