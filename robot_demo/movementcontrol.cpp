#include "movementcontrol.h"
#include <pthread.h>
#include <math.h>
#define P_REG 2.5
#define P_ANG 2.0
#define ANGL_DZ 1
#define POS_DZ 60

#define SPEED_SAT_UP 180
#define ANG_SAT_UP 180
#define ANG_SAT_DOWN 25
/*Public methods*/

MovementControl::MovementControl(float dt, iRobotCreate *robot) : Mapping(true) {

    this->dt = dt;
    irob_current_pose = POSITION();
    irob_desired_pose = POSITION();
    irob_start_pose = POSITION();

    new_pose = POSITION();

    this->robot = robot;

    this->irob_current_pose.x = -500;
    this->irob_current_pose.y = 500;
    this->irob_start_pose.x = -500;
    this->irob_start_pose.y = -500;

    speed_up=0;
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
    Mapping::movement_state = 2;
    this->robot->move(-speed,speed);
}

float MovementControl::robRotateL(DWORD speed){
    Mapping::movement_state = 2;
    this->robot->move(speed,-speed);

}

float MovementControl::robMove(DWORD speed){
    Mapping::movement_state = 1;
    this->robot->move(speed,speed);
}

void MovementControl::robStop(){

    Mapping::movement_state = 0;
    this->robot->move(0,0);
}

void MovementControl::updatePose(float pose_change, float angle_change){

    float angle=0;

    //todo teting of remake switch statement to if statement
    if(Mapping::movement_state == 1 || pose_change != 0){

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

    if(Mapping::movement_state == 2 || angle_change != 0){

        if(this->irob_current_pose.angle >=180){
            this->irob_current_pose.angle = -180 + angle_change;
        }
        else if(this->irob_current_pose.angle <=-180){
            this->irob_current_pose.angle = 180 + angle_change;
        }
        else{
            this->irob_current_pose.angle = this->irob_current_pose.angle + angle_change;
        }
        dist_sum = 0;
        irob_start_pose.x = this->irob_current_pose.x;
        irob_start_pose.y = this->irob_current_pose.y;
    }

    if(Mapping::movement_state == 0 && pose_change == 0 && angle_change == 0){

        dist_sum = 0;
        irob_start_pose.x = this->irob_current_pose.x;
        irob_start_pose.y = this->irob_current_pose.y;

    }

    std::cout << "angle" << this->irob_current_pose.angle   << std::endl;
    std::cout << "Suradnice X: " << this->irob_current_pose.x << " Suradnice Y" << this->irob_current_pose.y << std::endl;

    // poslatie pozicie do podedenej classy
    pthread_mutex_lock (&current_pose_lock);

    irob_current_mapping_pose = irob_current_pose;

    pthread_mutex_unlock (&current_pose_lock);
}

void MovementControl::moveToNewPose(float speed){

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

        //std::cout << "TEMP ANGLE " << temp_angle << std::endl;

        if (fabs(temp_angle)<=ANGL_DZ){
            //this->robot->move(0,0);
            this->robStop();
            speed_up=0;
            setPosAngle(true);
            return true;

        }

        else if (temp_angle>0) {
            cur_speed=fabs(temp_angle)*P_ANG;
            if (cur_speed-speed_up>10){
                cur_speed=speed_up+10;}


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
            if (cur_speed-speed_up>10){
                cur_speed=speed_up+10;}


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
            speed_uppos=0;
            return true;
        }
        else{
            if (fabs(temp_angle)>=ANGL_DZ && (temp_dist > 250)){
                speed_uppos=0;
                setPosAngle(false);
            }
           /*if (temp_angle>(ANGL_DZ)){this->robRotateR(15); speed_uppos=15;  std::cout << "TEMP ANGLE R" << temp_angle << std::endl;}
            else if (temp_angle<-ANGL_DZ){this->robRotateL(15); speed_uppos=15;  std::cout << "TEMP ANGLE L" << temp_angle << std::endl;}
            else {*/
                cur_speed=temp_dist*P_REG;
                if (cur_speed-speed_uppos>20){
                    cur_speed=speed_uppos+20;}


                if(cur_speed>SPEED_SAT_UP){
                    cur_speed=SPEED_SAT_UP;
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
