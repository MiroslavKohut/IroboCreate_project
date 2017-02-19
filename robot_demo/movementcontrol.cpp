#include "movementcontrol.h"
#include <math.h>

/*Public methods*/
MovementControl::MovementControl(float dt)
{
    this->dt = dt;
    irob_current_pose = POSITION();
    irob_desired_pose = POSITION();

}

MovementControl::~MovementControl()
{

}

void MovementControl::updatePose(float pose_change, float angle_change){

    this->irob_current_pose.x = this->irob_current_pose.x + pose_change*cos(angle_change);
    this->irob_current_pose.y = this->irob_current_pose.y + pose_change*sin(angle_change);

}

void MovementControl::setNewPoint(float speed, POSITION pose){

}



/*Private methods*/
float MovementControl::comuteAngle(){

}

float MovementControl::comuteTranslation(){

}

float MovementControl::pidControlRotation(float actual_angle, float desired_movement){

}

float MovementControl::pidControlTranslation(float actual_pose, float desired_movement){

}


