#include "movementcontrol.h"
#include <math.h>

#define angl_dz 5
#define pos_dz 5
/*Public methods*/
MovementControl::MovementControl(float dt, iRobotCreate & robot)
{
    this->dt = dt;
    irob_current_pose = POSITION();
    irob_desired_pose = POSITION();
    this->robot=robot;
    speed_up=0;
    speed_sat=35;

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
    if(this->pidControlRotation()){
        this->pidControlTranslation();
    }
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

    float temp_angle;

    if (curr_pos_reach){return true;}
    else{
    temp_angle=MovementControl::comuteAngle();
    if (fabs(temp_angle)<angl_dz){
        robot.move(0,0);
        speed_up=0;
        return true;
    }
    else if (temp_angle>0) {
        robot.move(speed_up,-speed_up);
        speed_up+=5;
    }
    else if (temp_angle<0) {
        robot.move(-speed_up,speed_up);
        speed_up+=5;
    }
    if (speed_up>speed_sat){
        speed_up=speed_sat;
    }


}
}

bool MovementControl::pidControlTranslation(){

    float temp_dist;
    if (curr_pos_reach){return true;}
    else{
        temp_dist=comuteTranslation();

        if (fabs(temp_dist)<pos_dz){
            robot.move(0,0);
            speed_uppos=0;
            return true;
        }
        else {
            robot.move(speed_uppos,speed_uppos);
            speed_uppos+=5;
        }
        if (speed_uppos>speed_sat){
            speed_uppos=speed_sat;
        }
    }
}


