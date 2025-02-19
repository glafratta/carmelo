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


//void Configurator::done_that(vertexDescriptor& src, bool & plan_works, b2World & world, std::vector<vertexDescriptor>& plan_provisional){
	// 	//std::pair<bool, vertexDescriptor> been(false, TransitionSystem::null_vertex());
	// 	//was ve instead of src
	// 	std::vector <vertexDescriptor> options_src;
    //     //IF THE CURRENT PLAN HAS EXECUTED
	// 	//if (bool fin=controlGoal.checkEnded(transitionSystem[src], UNDEFINED, true).ended; fin ){ //&& currentVertex!=movingVertex
	// 	//printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
	// 		std::vector <BodyFeatures> b_features=worldBuilder.getFeatures(data2fp, b2Transform(b2Vec2(0,0), b2Rot(0)), currentTask.direction, BOX2DRANGE);
	// 		//Disturbance where=controlGoal.disturbance;
	// 		State s_temp;
	// 		if (!b_features.empty()){
	// 			s_temp.Dn= Disturbance(b_features[0]); //assumes 1 item length
	// 			bool closest_match=1;
	// 			findMatch(s_temp,transitionSystem, transitionSystem[movingEdge.m_source].ID, UNDEFINED, StateMatcher::DISTURBANCE, &options_src);
	// 			//FIND STATE WHICH matches the relationship with the disturbance
	// 		}
	// 		//been= been_there(transitionSystem, where); 
	// 	//}
	// 	//printf("checked been = %i\n", been.first);

	// 	//std::vector <vertexDescriptor> plan_provisional=planVertices;
	// //	if (been.first){
	// 	//	printf("provisional plan\n");
	// 	for (auto o:options_src){
	// 		plan_provisional=planner(transitionSystem, o); //been.second, been.first
	// 		vertexDescriptor end =*(plan_provisional.rbegin().base()-1);
	// 		if (controlGoal.checkEnded(transitionSystem[end]).ended && checkPlan(world, plan_provisional, transitionSystem, s_temp.Dn)){
	// 			plan_works=true;
	// 			break;
	// 		}
	// 	}

//}

std::vector <BodyFeatures> WorldBuilder::processData(const CoordinateContainer& points, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <Pointf> ptset= set2vec(points);
    std::pair<bool,BodyFeatures> feature= bounding_box(ptset);
    if (feature.first){
        feature.second.pose.q.Set(start.q.GetAngle());
        result.push_back(feature.second);
    }
    return result;
}


#endif