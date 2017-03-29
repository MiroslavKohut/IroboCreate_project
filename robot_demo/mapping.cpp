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

//
//50cm x 50cm start point // 600cm x 600cm area // 10cm x 10cm square // 60*60 array
using namespace std;


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
}

Mapping::~Mapping()
{
    //pthread_join(mapping_thread,NULL);
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

void Mapping::startMapping(){

    if(!getMappingStatus()){
        setMappingStatus(true);
        thread_id=pthread_create(&mapping_thread,NULL,&mappingThreadFun,(void *)this);
    }
    return;
}

void Mapping::stopMapping(){

    setMappingStatus(false);
    pthread_join(mapping_thread,NULL);

}

float degTorad(float data){
    return data * RAD_DEG;
}

int Mapping::getPoints(){
    
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
                if(dist > 2000)
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
                /*if (point.x <0 ){
                    point.x = 0;
                }
                if (point.y <0){
                    point.y = 0;
                }
                if (point.y > 5000){
                    cout << point.y << endl;
                    point.y = 4900;
                }
                if (point.x > 5000){
                    cout << point.x << endl;
                    point.x = 4900;
                }*/

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
    //TODO TEST DATA FROM thread in points vector;
    std::cout << "zmapoval som" << std::endl;
    usleep(1000000);
    }
    return 0;

}

void Mapping::createDynamicMap(POINT bod){

    //pthread_mutex_lock (&mapppin_mutex);
    map[(int)(bod.x/DIV_CONST)][(int)(bod.y/DIV_CONST)] = 1;
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
               if(d[n]=='.'){
                   d[n]=',';
               }
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
                  for(int j=-2;j<3;j++){
                      for(int m=-2;m<3;m++){
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
               for(int j=-2;j<3;j++){
                   for(int m=-2;m<3;m++){
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
        else if (map[end_x][end_y] > 1){

            POINT curr_array_pose;
            bool previous_x =false;
            bool previous_y =false;

            int end_number = map[end_x][end_y];

            curr_array_pose.x = end_x;
            curr_array_pose.y = end_y;

            map[end_x][end_y] = 200;

            cesta.push_back(curr_array_pose);

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

                }

                x_pose = end_x+1;
                y_pose = end_y;
                if(map[x_pose][y_pose] < end_number && map[x_pose][y_pose] > 2 && !jump){
                    jump = true;
                    end_x++;
                }

                x_pose = end_x;
                y_pose = end_y+1;
                if(map[x_pose][y_pose] == end_number && map[x_pose][y_pose] > 2 && !jump){
                   jump = true;
                   end_y++;
                }
/*
                if(!jump){
                    end_number++;
                    if(end_x == start_x && end_y == start_y){
                        map[end_x][end_y] = 200;
                        curr_array_pose.x = end_x;
                        curr_array_pose.y = end_y;
                        cesta.push_back(curr_array_pose);
                        break;
                    }
                    continue;
                }*/

                map[end_x][end_y] = 200;
                curr_array_pose.x = end_x;
                curr_array_pose.y = end_y;
                cesta.push_back(curr_array_pose);
                end_number--;

            }

            cout << "PATH FOUND" << endl;
            cout << "SIZE: " << cesta.size() << endl;
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



bool Mapping::clearMap(){

    for(int x=0;x < MAP_WIDTH ;x++){
        for(int y=0;y < MAP_HIGHT;y++){
            map[x][y]=0;
         }
    }
}



