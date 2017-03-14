#include "movementcontrol.h"
#include <math.h>
#define P_REG 1.5
#define P_ANG 2.0
#define ANGL_DZ 1
#define POS_DZ 60
#define RAD_DEG M_PI/180
#define DEG_RAD 180/M_PI
#define ANG_SAT_UP 400
#define ANG_SAT_DOWN 25
/*Public methods*/

MovementControl::MovementControl(float dt, iRobotCreate *robot)
{
    this->dt = dt;
    irob_current_pose = POSITION();
    irob_desired_pose = POSITION();
    irob_start_pose = POSITION();
    new_pose = POSITION();

    this->robot = robot;

    movement_state = 0; //1 movement 2 rotation

    speed_up=0;
    speed_uppos = 0;
    speed_sat=400;
    pos_reach=true;
    ang_reach=true;
    dist_sum = 0;
    target_angle=0;

}

MovementControl::~MovementControl()
{

}

float MovementControl::degTorad(float data){
    return data * RAD_DEG;
}

float MovementControl::radTodeg(float data){
    return data * DEG_RAD;
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

    //todo teting of remake switch statement to if statement

    if(movement_state == 1 || pose_change > 0){

        if (this->irob_current_pose.angle  <= 90)
            angle = abs(this->irob_current_pose.angle  - 90);
        else if(this->irob_current_pose.angle  > 90 && this->irob_current_pose.angle  <= 180){
            angle = 450 - abs(this->irob_current_pose.angle);
        }
        else if(this->irob_current_pose.angle  < 0 && this->irob_current_pose.angle  >= -180){
            angle = 90 + abs(this->irob_current_pose.angle);
        }
        dist_sum = dist_sum + pose_change;


        this->irob_current_pose.x = irob_start_pose.x + dist_sum * cos(degTorad(angle));
        this->irob_current_pose.y = irob_start_pose.y + dist_sum * sin(degTorad(angle));
        //std::cout << "istance: " << dist_sum << "start" << irob_start_pose.x << std::endl;
    }

    if(movement_state == 2 || angle_change > 0){

        this->irob_current_pose.angle = this->irob_current_pose.angle + angle_change;

    }

    if(movement_state == 0 && pose_change == 0 && angle_change == 0){

        dist_sum =0;
        irob_start_pose.x = this->irob_current_pose.x;
        irob_start_pose.y = this->irob_current_pose.y;

    }

    std::cout << "angle" << this->irob_current_pose.angle   << std::endl;

    std::cout << "Suradnice X: " << this->irob_current_pose.x << " Suradnice Y" << this->irob_current_pose.y << std::endl;

}

void MovementControl::moveToNewPose(float speed){

    /*if (new_pose.angle == -180)
        new_pose.angle = -179.8; TODO dorobit*/
    this->irob_desired_pose = new_pose;
    if(ang_reach){
        if(this->pidControlTranslation()){
            setPosReach(true);
        }
    }
    else{
         this->pidControlRotation();
    }

}



/*Private methods*/
float MovementControl::comuteAngle(){

   float x =  this->irob_desired_pose.x - this->irob_current_pose.x;
   float y =  this->irob_desired_pose.y - this->irob_current_pose.y;

   float angle_from_y;

   if(x== 0 & y == 0){
      angle_from_y = 0;
   }
   else if (y >= 0){
       angle_from_y = 90 - radTodeg(atan2(y,x));

       //std::cout << "ATAN: " << radTodeg(atan2(y,x))<< std::endl;
   }
   else if (y < 0 && x >= 0){
       angle_from_y = fabs(-90 + radTodeg(atan2(y,x)));
   }
   else if(y < 0 && x < 0){
       angle_from_y = - 90 - (180 + radTodeg(atan2(y,x)));
   }

   if (angle_from_y > 180){
       angle_from_y = -360+ angle_from_y;
   }
   else if(angle_from_y < -180){
       angle_from_y = 360 + angle_from_y;
   }

   if (irob_current_pose.angle  == 0)
   {
       //std::cout << "Uhol1: " << angle_from_y << std::endl;
       return angle_from_y;

   }
   else{
      //std::cout << "Uhol2: " << angle_from_y << std::endl;
       return angle_from_y - irob_current_pose.angle;
   }

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

    if (pos_reach || ang_reach){  std::cout << "JELITO " << temp_angle << std::endl; return true;}
    else{
        temp_angle=MovementControl::comuteAngle();

        std::cout << "TEMP ANGLE " << temp_angle << std::endl;

        if (fabs(temp_angle)<=ANGL_DZ){
            //this->robot->move(0,0);
            this->robStop();

            setPosAngle(true);
            return true;

        }

        else if (temp_angle>0) {
            cur_speed=fabs(temp_angle)*P_ANG;
            if (cur_speed-speed_up>40){
                cur_speed=speed_up+40;}


            if(cur_speed>ANG_SAT_UP){
                cur_speed=ANG_SAT_UP;
            }
            if(cur_speed<ANG_SAT_DOWN){
                cur_speed=ANG_SAT_DOWN;
            }
             speed_up=cur_speed;
            //this->robot->move(-(DWORD)cur_speed,(DWORD)cur_speed);
            this->robRotateR(cur_speed);

        }

        else if (temp_angle<0) {
            cur_speed=fabs(temp_angle)*P_ANG;
            if (cur_speed-speed_up>40){
                cur_speed=speed_up+40;}


            if(cur_speed>ANG_SAT_UP){
                cur_speed=ANG_SAT_UP;
            }
            if(cur_speed<ANG_SAT_DOWN){
                cur_speed=ANG_SAT_DOWN;
            }
            speed_up=cur_speed;
            this->robRotateL(cur_speed);

        }
       //target_angle=this->irob_current_pose.angle;
        return false;
    }
}
bool MovementControl::pidControlTranslation(){

    float cur_speed;
    float temp_dist;
    float temp_angle;

    temp_dist=comuteTranslation();
    temp_angle=MovementControl::comuteAngle();

    if (pos_reach){
        return true;
    }
    else{

        std::cout << "DISTANCE: " << temp_dist << std::endl;

        if (fabs(temp_dist)<POS_DZ){
            //this->robot->move(0,0);
            this->robStop();

            return true;
        }
        else{
            if (fabs(temp_angle)>=ANGL_DZ && (temp_dist > 1300)){
                setPosAngle(false);
            }
           /*if (temp_angle>(ANGL_DZ)){this->robRotateR(15); speed_uppos=15;  std::cout << "TEMP ANGLE R" << temp_angle << std::endl;}
            else if (temp_angle<-ANGL_DZ){this->robRotateL(15); speed_uppos=15;  std::cout << "TEMP ANGLE L" << temp_angle << std::endl;}
            else {*/
                cur_speed=temp_dist*P_REG;
                if (cur_speed-speed_uppos>50){
                    cur_speed=speed_uppos+50;}


                if(cur_speed>speed_sat){
                    cur_speed=speed_sat;
                }
             speed_uppos=cur_speed;
            this->robMove(cur_speed);
            //this->robot->move((DWORD)cur_speed,(DWORD)cur_speed);

            //}
        }

        return false;

    }
}

void MovementControl::setPosReach(bool pos_reach_){
    this->pos_reach = pos_reach_;
}
void MovementControl::setPosAngle(bool ang_reach_){
    this->ang_reach = ang_reach_;
}
