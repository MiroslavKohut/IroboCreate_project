#include "mapping.h"
#include <math.h>
#include "rplidar.h"
#include <iostream>
#include <fstream>

#include <string>
#include <cstring>
#include <vector>

#include <fstream>
#include <algorithm>
#include <iterator>
#define LOOK_DISTANCE 1000
#define LOOK_DIST_BUG 250


//
//50cm x 50cm start point // 600cm x 600cm area // 10cm x 10cm square // 60*60 array
using namespace std;
int a=0;
int min_angle;

Mapping::Mapping(bool with_scan)
{
    if(with_scan){
        lidar.connect("/dev/laser");
        lidar.enable();
        lidar.start();
        movement_state = 0; //1 movement 2 rotation
    }

    irob_current_mapping_pose = POSITION();
    map.resize(MAP_WIDTH,vector<uint8_t>(MAP_HIGHT,0));
    mapping_run = false;
    navigation_run = false;

    navigation_data = NAVIGATION_DATA();
    navigation_output = NAVIGATION_OUTPUT();
    navigation_output.data_ready = false;
}

Mapping::~Mapping()
{
    //pthread_join(mapping_thread,NULL);
}

MAPPING_OUTPUT Mapping::getMappingOutput(){

    pthread_mutex_lock (&mapping_output_lock);
    MAPPING_OUTPUT data = mapping_output;
    pthread_mutex_unlock (&mapping_output_lock);
    return data;
}


void Mapping::setMappingOutput(MAPPING_OUTPUT data){

    pthread_mutex_lock (&mapping_output_lock);
    this->mapping_output = data;
    pthread_mutex_unlock (&mapping_output_lock);
    return;
}

void Mapping::setNavigationOutput(NAVIGATION_OUTPUT data){

    pthread_mutex_lock (&navigation_data_lock);
    this->navigation_output = data;
    pthread_mutex_unlock (&navigation_data_lock);
    return;
}

NAVIGATION_OUTPUT Mapping::getNavigationOutput(){

    pthread_mutex_lock (&navigation_data_lock);
    NAVIGATION_OUTPUT data = navigation_output;
    pthread_mutex_unlock (&navigation_data_lock);
    return data;
}


void Mapping::setNavigationData(NAVIGATION_DATA data){

    pthread_mutex_lock (&navigation_data_lock);
    this->navigation_data = data;
    pthread_mutex_unlock (&navigation_data_lock);
    return;
}

NAVIGATION_DATA Mapping::getNavigationData(){

    pthread_mutex_lock (&navigation_data_lock);
    NAVIGATION_DATA  data = navigation_data;
    pthread_mutex_unlock (&navigation_data_lock);
    return data;
}


bool Mapping::getMappingStatus(){

    pthread_mutex_lock (&mapping_status_lock);
    bool data = mapping_run;
    pthread_mutex_unlock (&mapping_status_lock);
    return data;
}

void Mapping::setMappingStatus(bool data){

    pthread_mutex_lock (&mapping_status_lock);
    mapping_run = data;
    pthread_mutex_unlock (&mapping_status_lock);
    return;
}


bool Mapping::getNavigationStatus(){

    pthread_mutex_lock (&navigation_status_lock);
    bool data = navigation_run;
    pthread_mutex_unlock (&navigation_status_lock);
    return data;
}

void Mapping::setNavigationStatus(bool data){

    pthread_mutex_lock (&navigation_status_lock);
    navigation_run = data;
    pthread_mutex_unlock (&navigation_status_lock);
    return;
}


void Mapping::startMapping(){

    if(!getMappingStatus()){
        setMappingStatus(false);
        setMappingStatus(true);
        thread_id=pthread_create(&mapping_thread,NULL,&mappingThreadFun,(void *)this);
    }
    return;
}

void Mapping::stopMapping(){

    setNavigationStatus(false);
    setMappingStatus(false);
    pthread_join(mapping_thread,NULL);

}

void Mapping::startNavigation(){

    if(!getNavigationStatus()){
        setNavigationStatus(false);
        setNavigationStatus(true);
        thread_id=pthread_create(&mapping_thread,NULL,&mappingThreadFun,(void *)this);
    }
    return;
}

void Mapping::stopNavigation(){

    setMappingStatus(false);
    setNavigationStatus(false);
    pthread_join(mapping_thread,NULL);
    return;
}

float degTorad(float data){
    return data * RAD_DEG;
}

inline void Mapping::mappingLoop(){

    float angle=0;
    POINT point;
    POSITION current_pose;

    while(getMappingStatus()){

        if (movement_state != 2){

            LaserMeasurement measure=lidar.getMeasurement();

            pthread_mutex_lock (&current_pose_lock);
            current_pose = irob_current_mapping_pose;
            pthread_mutex_unlock (&current_pose_lock);

            points.begin();
            for(int i=0; i<measure.numberOfScans;i++)
            {
                float dist = measure.Data[i].scanDistance *16+4.7;
                if(dist > 2000 || dist<200 || (dist>600 && dist<800))
                    continue;

                if (measure.Data[i].scanAngle <= 90)
                    angle = fabs(measure.Data[i].scanAngle - 90);
                else { //(measure.Data[i].scanAngle > 90 && measure.Data[i].scanAngle <= 360)
                    angle = 450 - measure.Data[i].scanAngle;

                }

                // robot angle usage
                if(current_pose.angle > 0){
                    angle = angle + 360 - current_pose.angle;
                }
                else if(current_pose.angle < 0){
                    angle = angle - current_pose.angle;
                }

                point.x = -(cos(degTorad(angle))* dist + current_pose.x);
                point.y = sin(degTorad(angle))* dist + current_pose.y;

                if (point.x < 0 ){
                    point.x = 0;
                }
                if (point.y < 0 ){
                    point.y = 0;
                }
                if(point.x < 4950 && point.y < 4950)
                    this->createDynamicMap(point);
                //TODO test map creation and add angle commutation
                points.push_back(point);
            }
        }
        else{
            usleep(10000);
            continue;
        }

//TODO test if map is reloaded properly

        MAPPING_OUTPUT data = getMappingOutput();

        map_zaloha.clear();
        map_zaloha=map;

        if(findPath(data.new_maping_pose,data.start_point,data.end_point)){
            map.clear();
            map=map_zaloha;
            data.data_ready = true;
            setMappingOutput(data);
        }
        else{
            std::cout << "NEVIEM NAJST PATH" << std::endl;
            POINT data_set = data.end_point;
            data_set.x = data_set.x/-50;
            data_set.y = data_set.y/50;
            data.new_maping_pose.push_back(data_set);
            data.data_ready = true;
            setMappingOutput(data);
        }
        //TODO TEST DATA FROM thread in points vector;
        std::cout << "zmapoval som" << std::endl;
        usleep(300000);
    }
    return;
}

inline void Mapping::navigationLoop(){
    float target_angle;
    float angle=0;
    POINT point;
    POSITION current_pose;
    NAVIGATION_DATA data_navigacie;
    NAVIGATION_OUTPUT vystup_navigacie;

    while(getNavigationStatus()){
        vystup_navigacie.data_ready=false;
        vystup_navigacie.front_view_block=false;
        data_navigacie = getNavigationData();


        if(data_navigacie.goal_angle<0)
            target_angle=data_navigacie.goal_angle+360;
        else
            target_angle=data_navigacie.goal_angle;

        if (movement_state != 2){

            LaserMeasurement measure=lidar.getMeasurement();

            pthread_mutex_lock (&current_pose_lock);
            current_pose = irob_current_mapping_pose;
            pthread_mutex_unlock (&current_pose_lock);

            vystup_navigacie.everything_blocked=false;
            vystup_navigacie.clear_path_to_goal=true;

            for(int i=0; i<measure.numberOfScans;i++)
            {
                float temp=measure.Data[i].scanAngle - target_angle;

                if (target_angle<45 && measure.Data[i].scanAngle>360-(45-target_angle)){
                    temp=-360+temp;
                }
                if (target_angle>315 && measure.Data[i].scanAngle<(45-(360-target_angle))){
                    temp=360+temp;
                     //cout << "temp is " << temp << endl;
                }

                /*cout << "measured_angle " << measure.Data[i].scanAngle << endl;
                cout << "target_angle_angle " << target_angle << endl;
                cout << "temp " << temp << endl;*/

                // pri vzd LOOK_DISTANCE a sirke 400 sa nesmu data objavit v rozmedzi uhla 30°

                float dist = measure.Data[i].scanDistance *16+4.7;
                if(dist<150)
                    continue;
                if(temp>-45 && temp<45){
                  //  cout << "valid angle " << measure.Data[i].scanAngle << endl;
                    if(temp>=0){
                        float temp_rect_dist=225/sin(degTorad(temp));
                        if (temp_rect_dist>LOOK_DISTANCE){
                            temp_rect_dist=LOOK_DISTANCE;
                        }
                        //cout << " at + angle " << measure.Data[i].scanAngle << " distance " << dist << " expected " << temp_rect_dist << endl;
                        if(temp_rect_dist>dist){
                         // cout << "blocked @" << endl;
                            vystup_navigacie.clear_path_to_goal=false;
                    }
                        else {
                            //KED NIE JE PREKAZKA
                        }

                    }
                    if(temp<0){
                        float temp_rect_dist=225/sin(degTorad(fabs(temp)));
                        if (temp_rect_dist > LOOK_DISTANCE)
                        {
                            temp_rect_dist=LOOK_DISTANCE;
                        }
                        //cout << " at - angle " << measure.Data[i].scanAngle << " distance " << dist << " expected " << temp_rect_dist << endl;
                        if(temp_rect_dist>dist){
                        //    cout << "blocked @" << endl;
                            vystup_navigacie.clear_path_to_goal=false;
                    }
                        else {
                            //KED NIE JE PREKAZKA
                        }

                    }
                }

//                angles.push_back(measure.Data[i].scanAngle);


            }


            ///// HALDANIE NOVEHO UHLA /////////////
            if(vystup_navigacie.clear_path_to_goal==false){
                float temp_target;
                min_angle=360;
                bool temp_free_front=true;

                for (int l=0;l<360;l+=4){
                    bool temp_free_path=true;
                            ;
                  if(data_navigacie.goal_angle<0)
                        temp_target=0;//data_navigacie.goal_angle+360;
                    else
                        temp_target=0;//data_navigacie.goal_angle;

                  float target_angle2=l;



                    for(int i=0; i<measure.numberOfScans;i++)
                    {
                        float temp=measure.Data[i].scanAngle-target_angle2;

                        if (target_angle2<45 && measure.Data[i].scanAngle>360-(45-target_angle2)){
                            temp=-360+temp;
                        }
                        if (target_angle2>315 && measure.Data[i].scanAngle<(45-(360-target_angle2))){
                            temp=360+temp;
                             //cout << "temp is " << temp << endl;
                        }

                        // pri vzd LOOK_DISTANCE a sirke 400 sa nesmu data objavit v rozmedzi uhla 30°

                        float dist = measure.Data[i].scanDistance *16+4.7;
                        if(dist<150)
                            continue;
                        if( temp>-45 && temp<45 ){

                            if(temp>=0){
                                float temp_rect_dist=225/sin(degTorad(temp));
                                if(!data_navigacie.bug_enabled){
                                    if (temp_rect_dist>LOOK_DISTANCE)
                                    {
                                        temp_rect_dist=LOOK_DISTANCE;
                                    }
                                }
                                else{
                                    if (temp_rect_dist>LOOK_DIST_BUG)
                                    {
                                        temp_rect_dist=LOOK_DIST_BUG;
                                    }
                                }
                                if(temp_rect_dist>dist){
                                    temp_free_path=false;
                            }
                                else {
                                    //KED NIE JE PREKAZKA
                                }


                            }
                            if(temp<0){
                                float temp_rect_dist=225/sin(degTorad(fabs(temp)));
                                if(!data_navigacie.bug_enabled){
                                    if (temp_rect_dist>LOOK_DISTANCE)
                                    {
                                        temp_rect_dist=LOOK_DISTANCE;
                                    }
                                }
                                else{
                                    if (temp_rect_dist>LOOK_DIST_BUG)
                                    {
                                        temp_rect_dist=LOOK_DIST_BUG;
                                    }
                                }
                                if(temp_rect_dist>dist){
                                   temp_free_path=false;
                            }
                                else {
                                    //KED NIE JE PREKAZKA
                                }

                            }
                        }

                        if (l==0){
                            if( temp>-45 && temp<45 ){

                                if(temp>=0){
                                    float temp_rect_dist=150/sin(degTorad(temp));
                                    if(!data_navigacie.bug_enabled){
                                        if (temp_rect_dist>LOOK_DISTANCE)
                                        {
                                            temp_rect_dist=LOOK_DISTANCE;
                                        }
                                    }
                                    else{
                                        if (temp_rect_dist>LOOK_DIST_BUG)
                                        {
                                            temp_rect_dist=LOOK_DIST_BUG;
                                        }
                                    }

                                    if(temp_rect_dist>dist){
                                        temp_free_front=false;
                                }
                                    else {
                                        //KED NIE JE PREKAZKA
                                    }





                                }
                                if(temp<0){
                                    float temp_rect_dist=150/sin(degTorad(fabs(temp)));
                                    if(!data_navigacie.bug_enabled){
                                        if (temp_rect_dist>LOOK_DISTANCE)
                                        {
                                            temp_rect_dist=LOOK_DISTANCE;
                                        }
                                    }
                                    else{
                                        if (temp_rect_dist>LOOK_DIST_BUG)
                                        {
                                            temp_rect_dist=LOOK_DIST_BUG;
                                        }
                                    }
                                    if(temp_rect_dist>dist){
                                       temp_free_front=false;
                                }
                                    else {
                                        //KED NIE JE PREKAZKA
                                    }

                                }
                            }

                        }

                }
                    float angle_distance;
                    vystup_navigacie.front_view_block=!temp_free_front;

                    if (!data_navigacie.bug_enabled){
                        angle_distance=fabs(target_angle2-temp_target);

                        if (angle_distance>180){
                            angle_distance=360-angle_distance;
                        }
                    }
                    else {angle_distance=l;
                     }

//
                if(fabs(angle_distance)<min_angle && temp_free_path==true){
                    a=l;
                    min_angle=angle_distance;
                    //cout <<  "min angle "<< min_angle <<" at angle " << vystup_navigacie.new_angle  << endl;
                    vystup_navigacie.new_angle=target_angle2;
                }
            }
                //min_angle=360;

        }

            if (vystup_navigacie.clear_path_to_goal==false){
                cout <<  "nearest_angle cw @  "<< vystup_navigacie.new_angle  << endl;

            }    

    }
    if(min_angle>90 && !data_navigacie.bug_enabled){
        vystup_navigacie.everything_blocked=true;
        cout << "@@@@@@@@@@@@@@@@@@@@EVERYTHING BLOCKED @@@@@@@@@@@@@@@@"<< endl;
    }
        ////////////////////////////HLADANIE NOVEHO UHLA///////////////

    //TODO TEST DATA FROM thread in points vector;
    std::cout << "Navigujem" << std::endl;
    cout << "TARGET_ANGLE " << target_angle << endl;
    cout << "VYSTUP NAVIGACIE " << vystup_navigacie.clear_path_to_goal << endl;
    cout << "Front View Blocked " << vystup_navigacie.front_view_block << endl;



    vystup_navigacie.data_ready=true;

    if (vystup_navigacie.clear_path_to_goal)
        vystup_navigacie.new_angle = -1;

    setNavigationOutput(vystup_navigacie);

    usleep(100000);
    }
    setNavigationOutput(vystup_navigacie);
    return;
}

int Mapping::getPoints(){

    //lool controlling mapping of the robot
    Mapping::mappingLoop();

    // loop controlling navigation of the robot
    Mapping::navigationLoop();

    return 0;

}

void Mapping::createDynamicMap(POINT bod){

    //pthread_mutex_lock (&mapppin_mutex);

    int x = (int)(bod.x/DIV_CONST);
    int y = (int)(bod.y/DIV_CONST);
    map[x][y] = 1;

    for(int j=-3;j<=3;j++){
        for(int m=-3;m<=3;m++){
            if (x+j > 0 && x+j <MAP_WIDTH && y+m > 0 && x+m < MAP_HIGHT)
              map[x+j][y+m]= 1;
        }
    }
    //pthread_mutex_unlock (&mapppin_mutex);


}

void Mapping::loadFile(){


   //TODO VYMENIT X za Y
   int i=0;
   int pos[40];
   int y=0;
   int n;
   int slope;
   double temp;
   int k;
   int total_n=0;
   int p;
   string line;

   ifstream myfile ("priestor.txt");
   if (myfile.is_open())
      {
       std::cout << "asd"<<std::endl;
        while ( getline (myfile,line) )
        {
           y=0;
           const char * c = line.c_str();
           char* d=(char*) c;
           for (n=0;n<strlen(d);n++){
               if(d[n]==','){
                   d[n]='?';
               }
               /*if(d[n]=='.'){
                   d[n]=',';
               }*/
           }
           d++;
           for (n=0;n<(c[0]-48)*2;n++){

                while (*d && !(isdigit(*d))){
                        d++;
                }
                temp=strtod(d, &d);
                pos[y]=round(temp);
                y++;
                 std::cout << temp<<std::endl;
           }
           for (n=0;n<y;n++){
               pos[n]=pos[n]*2;
           }

           for (n=0;n<y-2;n+=2){
              int distx=(pos[n+2]-pos[n]);
              int disty=(pos[n+3]-pos[n+1]);

              for (k=0;k<160;k++){
                  for(int j=-3;j<=3;j++){
                      for(int m=-3;m<=3;m++){
                          if((uint8_t)round(pos[n+1]/10+disty*k/1600)+j>0 &&(uint8_t)round(pos[n]/10+distx*k/1600)+m>0){
                            map[(uint8_t)round(pos[n+1]/10+disty*k/1600)+j][(uint8_t)round(pos[n]/10+distx*k/1600)+m] = 1;
                          }
                      }
                  }
              }
           }
           int distx=(pos[0]-pos[n]);
           int disty=(pos[1]-pos[n+1]);

           for (k=0;k<160;k++){
               for(int j=-3;j<=3;j++){
                   for(int m=-3;m<3;m++){
                       if((uint8_t)round(pos[n+1]/10+disty*k/1600)+j>0 &&(uint8_t)round(pos[n]/10+distx*k/1600)+m>0){
                         map[(uint8_t)round(pos[n+1]/10+disty*k/1600)+j][(uint8_t)round(pos[n]/10+distx*k/1600)+m] = 1;
                       }
                   }
               }
           }


        }
        myfile.close();
      }
}

bool Mapping::findPath(std::vector<POINT> &cesta, POINT start, POINT end){

    start.x = -start.x;
    end.x = -end.x;

    std::vector<POINT> final_path;
    cesta.clear();

    uint8_t start_num = 2;
    uint8_t start_x =(int)start.x/DIV_CONST;
    uint8_t start_y= (int)start.y/DIV_CONST;

    uint8_t end_x =(int)end.x/DIV_CONST;
    uint8_t end_y= (int)end.y/DIV_CONST;

    uint8_t x_pose;
    uint8_t y_pose;

    map[start_x][start_y] = start_num;


    while(1){
        for(int x=1;x < MAP_WIDTH-1 ;x++){
            for(int y=1;y < MAP_HIGHT-1;y++){

                if(map[x][y]== start_num){
                   x_pose = x+1;
                   y_pose = y;
                   if(map[x_pose][y_pose] == 0)
                       map[x_pose][y_pose] = start_num + 1;

                   x_pose = x-1;
                   y_pose = y;
                   if(map[x_pose][y_pose] == 0)
                       map[x_pose][y_pose] = start_num + 1;

                   x_pose = x;
                   y_pose = y-1;
                   if(map[x_pose][y_pose] == 0)
                       map[x_pose][y_pose] = start_num + 1;

                   x_pose = x;
                   y_pose = y+1;
                   if(map[x_pose][y_pose] == 0)
                       map[x_pose][y_pose] = start_num + 1;
               }
            }
        }

        if(map[end_x][end_y] == 1){
            cout << "PATH IS COLLIDES WITH SURROUNDING" << endl;
            return false;
        }
        // NASLO KONCOVY BOD IDEM NAJST CESTU
        else if (map[end_x][end_y] > 1){

            POINT curr_array_pose;
            bool previous_x =false;
            bool previous_y =false;

            int end_number = map[end_x][end_y];

            curr_array_pose.x = end_x;
            curr_array_pose.y = end_y;

            map[end_x][end_y] = 201;

            cesta.push_back(curr_array_pose);
            final_path.push_back(curr_array_pose);

            while(end_number > 2){

                bool jump = false;

                if(previous_x){

                    x_pose = end_x;
                    y_pose = end_y-1;

                    if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                       jump = true;
                       end_y--;
                       previous_y = true;
                       previous_x = false;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                    x_pose = end_x;
                    y_pose = end_y+1;
                    if(map[x_pose][y_pose] == end_number && map[x_pose][y_pose] > 2 && !jump){
                       jump = true;
                       end_y++;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                    if(!jump){
                        x_pose = end_x;
                        y_pose = end_y-1;
                        if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                           jump = true;
                           previous_y = true;
                           previous_x = false;
                           end_y--;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }

                        x_pose = end_x-1;
                        y_pose = end_y;
                        if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump ){
                            jump = true;
                            previous_x = true;
                            previous_y = false;
                            end_x--;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }

                        x_pose = end_x+1;
                        y_pose = end_y;

                        if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                            jump = true;
                            end_x++;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }


                        x_pose = end_x;
                        y_pose = end_y+1;
                        if(map[x_pose][y_pose] == end_number && map[x_pose][y_pose] > 2 && !jump){
                           jump = true;
                           end_y++;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }
                    }

                }
                else if(previous_y){

                    x_pose = end_x-1;
                    y_pose = end_y;

                    if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump ){
                        jump = true;
                        end_x--;
                        previous_x = true;
                        previous_y = false;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                    x_pose = end_x+1;
                    y_pose = end_y;
                    if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                        jump = true;
                        end_x++;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                    if(!jump){
                        x_pose = end_x;
                        y_pose = end_y-1;
                        if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                           jump = true;
                           previous_y = true;
                           previous_x = false;
                           end_y--;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }

                        x_pose = end_x-1;
                        y_pose = end_y;
                        if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump ){
                            jump = true;
                            previous_x = true;
                            previous_y = false;
                            end_x--;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }

                        x_pose = end_x+1;
                        y_pose = end_y;
                        if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                            jump = true;
                            end_x++;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }


                        x_pose = end_x;
                        y_pose = end_y+1;
                        if(map[x_pose][y_pose] == end_number && map[x_pose][y_pose] > 2 && !jump){
                           jump = true;
                           end_y++;
                        }
                        else if(!jump){
                            previous_y = false;
                            previous_x = false;
                        }
                    }
                }
                else
                {
                    x_pose = end_x;
                    y_pose = end_y-1;
                    if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                       jump = true;
                       previous_y = true;
                       previous_x = false;
                       end_y--;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                    x_pose = end_x-1;
                    y_pose = end_y;
                    if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump ){
                        jump = true;
                        previous_x = true;
                        previous_y = false;
                        end_x--;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                    x_pose = end_x+1;
                    y_pose = end_y;
                    if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                        jump = true;
                        end_x++;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }


                    x_pose = end_x;
                    y_pose = end_y+1;
                    if(map[x_pose][y_pose] == end_number && map[x_pose][y_pose] > 2 && !jump){
                       jump = true;
                       end_y++;
                    }
                    else if(!jump){
                        previous_y = false;
                        previous_x = false;
                    }

                }

                map[end_x][end_y] = 200;
                curr_array_pose.x = end_x;
                curr_array_pose.y = end_y;
                cesta.push_back(curr_array_pose);
                end_number--;

                if (end_number == 2)
                    map[end_x][end_y] = 201;
                    //break;
            }

            uint8_t mode=0;
            uint8_t global_mode=0;

            for (int i = 1;i<cesta.size();i++){

                if(map[(int)cesta[i].x+1][(int)cesta[i].y] == 200 && map[(int)cesta[i].x-1][(int)cesta[i].y] == 200 )
                    mode= 1;
                if(map[(int)cesta[i].x][(int)cesta[i].y+1] == 200 && map[(int)cesta[i].x][(int)cesta[i].y-1] == 200 )
                    mode= 2;
                if(map[(int)cesta[i].x][(int)cesta[i].y-1] == 200 && map[(int)cesta[i].x+1][(int)cesta[i].y] == 200 ||  map[(int)cesta[i].x-1][(int)cesta[i].y] == 200 && map[(int)cesta[i].x][(int)cesta[i].y+1] == 200) //doprava
                    mode= 3;
                if(map[(int)cesta[i].x-1][(int)cesta[i].y] == 200 && map[(int)cesta[i].x][(int)cesta[i].y-1] == 200 ||  map[(int)cesta[i].x+1][(int)cesta[i].y] == 200 && map[(int)cesta[i].x][(int)cesta[i].y+1] == 200) //dolava
                    mode= 4;
               /* printf("cesta y %d ",(int)cesta[i].y);
                printf("cesta x %d \n",(int)cesta[i].x);
                printf(" increment %d \n",i);*/

                if (global_mode != mode && i > 4){
                   /* printf(" velkost cesty %d",cesta.size());
                    printf(" increment %d \n",i);*/
                    map[(int)cesta[i].x][(int)cesta[i].y] = 201;
                    final_path.push_back(cesta[i]);

                }
                global_mode= mode;
            }
            curr_array_pose.x = end_x;
            curr_array_pose.y = end_y;
            final_path.push_back(curr_array_pose);
/*
            for (int i = 0 ; i< final_path.size();i++){
                 printf("cesta y %d ",(int)final_path[i].y);
                 printf("cesta x %d \n",(int)final_path[i].x);
            }*/

            //printf("Velkost cesty %d \n",final_path.size());
            //cout << "PATH FOUND" << endl;
            //cout << "SIZE: " << cesta.size() << endl;
            //cesta.clear();
            cesta = final_path;
            return true;
        }

        //cout << start_num << endl;
        start_num++;

        /*for(int x=0;x < MAP_WIDTH ;x++){
            for(int y=0;y < MAP_HIGHT;y++){
               printf("%3d ", map[x][y]);
               }
            cout << endl;
        }
        cout << endl;
        sleep(1);*/
    }
}

bool Mapping::filterPath(std::vector<POINT> &path){

    float k,q;
    int size = path.size()-1;
    for (int i = size-1; i >= 0 ; i++){
        k = path[size].y - path[i].y / path[size].x - path[i].x;
        q = path[size].y - k * path[size].x;
    }

}

bool Mapping::clearMap(){

    for(int x=0;x < MAP_WIDTH ;x++){
        for(int y=0;y < MAP_HIGHT;y++){
            map[x][y]=0;
         }
    }
}
