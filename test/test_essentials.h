#ifndef TEST_ESSENTIALS_H
#define TEST_ESSENTIALS_H
#include "configurator.h"
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip> 
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <ncurses.h>
#include <ctime>
#include <dirent.h>
#include <filesystem>
#define _USE_MATH_DEFINES


bool debug_draw(b2World & w, int file){
    char name_v[256], name_s[256], name_d[256];
    sprintf(name_v, "/tmp/debug_chassis_%i.txt",file);
    sprintf(name_s, "/tmp/debug_sensor_%i.txt",file);
    sprintf(name_d, "/tmp/debug_disturbance_%i.txt",file);
    FILE * f_v=fopen(name_v, "w");
    FILE * f_s=fopen(name_s, "w");
    FILE * f_d=fopen(name_d, "w");
    for (auto b=w.GetBodyList(); b; b=b->GetNext()){
        if (b->GetUserData().pointer){
            for (b2Fixture* fixture=b->GetFixtureList();fixture; fixture=fixture->GetNext()){
                b2PolygonShape * poly=(b2PolygonShape*)fixture->GetShape();
                int ct= poly->m_count;
                for (int i=0; i<ct; i++){
                    b2Vec2* v=poly->m_vertices+i;
                    b2Vec2 world_point= b->GetWorldPoint(*v);
                    if (b->GetUserData().pointer==ROBOT_FLAG){
                        if (fixture->IsSensor()){
                            fprintf(f_s, "%f\t%f\n",  world_point.x, world_point.y);            
                        }
                        else{
                            fprintf(f_v, "%f\t%f\n",  world_point.x, world_point.y);            
                        }
                    }
                    else if (b->GetUserData().pointer==DISTURBANCE_FLAG){
                        fprintf(f_d, "%f\t%f\n",  world_point.x, world_point.y);            
                    }
                }
            }
        }
    }
    fclose(f_v);
    fclose(f_s);
    fclose(f_d);
}



#endif