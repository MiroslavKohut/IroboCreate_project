#include "movementcontrol.h"
#include <math.h>
#define P_REG 10
#define ANGL_DZ 5
#define POS_DZ 5
#define RAD_DEG M_PI/180
/*Public methods*/
MovementControl::MovementControl(float dt, iRobotCreate *robot)
{
    this->dt = dt;
    irob_current_pose = POSITION();
    irob_desired_pose = POSITION();
    irob_start_pose = POSITION();

    this->robot = robot;

    movement_state = 0; //1 movement 2 rotation

    speed_up=10;
    speed_sat=10;
    pos_reach=true;
    dist_sum = 0;


}

MovementControl::~MovementControl()
{

}

float MovementControl::degTorad(float data){
    return data * RAD_DEG;
}

float MovementControl::robRotateR(DWORD speed){
    movement_state = 2;
    this->robot->move(-speed,speed);
}
float MovementControl::robRotateL(DWORD speed){
    movement_state = 2;
    this->robot->move(speed,-speed);

}

float MovementControl::robMove(DWORD speed){
    movement_state = 1;
    this->robot->move(speed,speed);
}

void MovementControl::robStop(){

    movement_state = 0;
    this->robot->move(0,0);
}



void MovementControl::updatePose(float pose_change, float angle_change){

    float angle=0;

    switch(movement_state){
        case 1 :

            if (this->irob_current_pose.angle  <= 90)
                angle = abs(this->irob_current_pose.angle  - 90);
            else if(this->irob_current_pose.angle  > 90 && this->irob_current_pose.angle  <= 180){
                angle = 450 - angle_change ;
            }
            else if(this->irob_current_pose.angle  < 0 && this->irob_current_pose.angle  >= -180){
                angle = 90 + abs(this->irob_current_pose.angle);
            }
            dist_sum = dist_sum + pose_change;


            this->irob_current_pose.x = irob_start_pose.x + dist_sum * cos(degTorad(angle));
            this->irob_current_pose.y = irob_start_pose.y + dist_sum * sin(degTorad(angle));
            std::cout << "istance: " << dist_sum << "start" << irob_start_pose.x << std::endl;

        break;

        case 2 :
            this->irob_current_pose.angle = this->irob_current_pose.angle + angle_change;
        break;

    default:{
        dist_sum =0;
        irob_start_pose.x = this->irob_current_pose.x;
        irob_start_pose.y = this->irob_current_pose.y;
    }

    }

    //std::cout << "UHOL: " << angle<< "pose" << pose_change  << std::endl;

    std::cout << "Suradnice X: " << this->irob_current_pose.x << " Suradnice Y" << this->irob_current_pose.y << std::endl;

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
      angle_from_y = -(atan(degTorad(x/y)) + 180);
   }
   else if (y > 0 && x > 0){
      angle_from_y = -(atan(degTorad(x/y)) - 180);
   }
   else
      angle_from_y = atan(degTorad(x/y));

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
            this->robot->move(0,0);
            speed_up=10;
            return true;
        }

        else if (temp_angle>0) {
            cur_speed=temp_angle*P_REG/speed_up;
            if(cur_speed>speed_sat){
                cur_speed=speed_sat;
            }
            this->robot->move(-(DWORD)cur_speed,(DWORD)cur_speed);
            speed_up-=1;
        }

        else if (temp_angle<0) {
            cur_speed=temp_angle*P_REG/speed_up;
            if(cur_speed>speed_sat){
                cur_speed=speed_sat;
            }

            this->robot->move((DWORD)cur_speed,-(DWORD)cur_speed);
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
            this->robot->move(0,0);
            speed_uppos=10;
            return true;
        }
        else {
            cur_speed=temp_dist*P_REG/speed_uppos;
            if(cur_speed>speed_sat){
                cur_speed=speed_sat;
            }
            this->robot->move((DWORD)cur_speed,(DWORD)cur_speed);
            speed_uppos-=1;
        }
        if (speed_uppos<=1){
            speed_uppos=1;
        }

    }
}

void MovementControl::setPosReach(bool pos_reach_){
    this->pos_reach = pos_reach_;
}

