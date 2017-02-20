#include "movementcontrol.h"
#include <math.h>
#define P_REG 10
#define ANGL_DZ 5
#define POS_DZ 5
/*Public methods*/
MovementControl::MovementControl(float dt, iRobotCreate & robot)
{
    this->dt = dt;
    irob_current_pose = POSITION();
    irob_desired_pose = POSITION();
    this->robot=robot;
    speed_up=10;
    speed_sat=10;
    pos_reach=true;
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
//predpokladame
    float temp_angle;
    float cur_speed;

    if (pos_reach){return true;}
    else{
    temp_angle=MovementControl::comuteAngle();
    if (fabs(temp_angle)<ANGL_DZ){
        robot.move(0,0);
        speed_up=10;
        return true;
    }
    else if (temp_angle>0) {
        cur_speed=temp_angle*P_REG/speed_up;
        if(cur_speed>speed_sat){
            cur_speed=speed_sat;
        }
        robot.move(-(DWORD)cur_speed,(DWORD)cur_speed);
        speed_up-=1;
    }
    else if (temp_angle<0) {
        cur_speed=temp_angle*P_REG/speed_up;
        if(cur_speed>speed_sat){
            cur_speed=speed_sat;
        }

        robot.move((DWORD)cur_speed,-(DWORD)cur_speed);
        speed_up-=1;
    }

    if (speed_up<=1){
        speed_up=1;
    }


}
}

bool MovementControl::pidControlTranslation(){

    float cur_speed;
    float temp_dist;
    if (pos_reach){return true;}
    else{
        temp_dist=comuteTranslation();

        if (fabs(temp_dist)<POS_DZ){
            robot.move(0,0);
            speed_uppos=10;
            return true;
        }
        else {
            cur_speed=temp_dist*P_REG/speed_uppos;
            if(cur_speed>speed_sat){
                cur_speed=speed_sat;
            }
            robot.move((DWORD)cur_speed,(DWORD)cur_speed);
            speed_uppos-=1;
        }
        if (speed_uppos<=1){
            speed_uppos=1;
        }

    }
}

void MovementControl::setPosReach(bool pos_reach_){
    pos_reach=pos_reach_;
}

//TODO setter current position reach
