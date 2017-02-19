#include "movementcontrol.h"
#include <math.h>

/*Public methods*/
MovementControl::MovementControl(float dt,iRobotCreate robot)
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
    this->irob_current_pose.angle = this->irob_current_pose.angle + angle_change;

}

void MovementControl::moveToNewPose(float speed, POSITION pose){

    this->irob_desired_pose = pose;
    if(this->pidControlRotation())
        this->pidControlTranslation();

}


/*Private methods*/
float MovementControl::comuteAngle(){

   float x = this->irob_current_pose.x - this->irob_desired_pose.x;
   float y = this->irob_current_pose.y - this->irob_desired_pose.y;

   float angle_from_y;

   if (x == 0 && y > 0){
       angle_from_y = -180;
   }
   else if (x == 0 && y < 0){
       angle_from_y = 0;
   }
   else if (y == 0 && x < 0){
       angle_from_y = 90;
   }
   else if (y == 0 && x > 0){
       angle_from_y = -90;
   }
   else if (y > 0 && x < 0){
      angle_from_y = -(atan(x/y) + 180);
   }
   else if (y > 0 && x > 0){
      angle_from_y = -(atan(x/y) - 180);
   }
   else
      angle_from_y = atan(x/y);

   return irob_current_pose.angle - angle_from_y;
}

float MovementControl::comuteTranslation(){

    float x = this->irob_current_pose.x - this->irob_desired_pose.x;
    float y = this->irob_current_pose.y - this->irob_desired_pose.y;

    return sqrt(x*x + y*y);

}

bool MovementControl::pidControlRotation(){



}

bool MovementControl::pidControlTranslation(){

}


