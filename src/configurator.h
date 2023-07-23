#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include "opencv2/opencv.hpp"
//#include "box2d/box2d.h"
//#include "robot.h"
#include <dirent.h>
#include <vector>
#include <thread>
#include <filesystem>
#include <cmath>
#include <unistd.h>
#include <ncurses.h>
#include <fstream>
//#include "task.h"
#include "general.h" //general functions + point class + typedefs + Primitive.h + boost includes
#include <algorithm>
#include <sys/stat.h>

typedef b2Transform DeltaPose;

class ConfiguratorInterface{
public:
	bool debugOn=0;
	int iteration=0;
	CoordinateContainer data;
	CoordinateContainer data2fp;
	bool ready=0;

	void setReady(bool b);

	bool isReady();
};

class Configurator{
protected:
	double samplingRate = 1.0/ 5.0; //default
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	b2Vec2 absPosition = {0.0f, 0.0f};
	FILE * dumpPath;
	char fileNameBuffer[50];
	Task currentTask;
public:
	ConfiguratorInterface * ci;
	bool running =0;
	std::thread * t=NULL;
	bool debugOn=0;
	float affineTransError =0;
	char *folder;
	char readMap[50];
	Task controlGoal;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	float totalTime=0;
	CoordinateContainer current, currentBox2D;
	bool planning =1;
	char statFile[100];
	bool timerOff=0;
	int bodies=0;
	int treeSize = 0; //for debug
	Sequence plan;
	M_CODES numberOfM =THREE_M;
	GRAPH_CONSTRUCTION graphConstruction = BACKTRACKING;

Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	totalTime =0.0f;
	if (debugOn){
		char dirName[50];
		sprintf(dirName, "bodiesSpeedStats");
		if (!opendir(dirName)){
			mkdir(dirName, 0777);
			printf("made stats directory\n");
		}
		else{
			printf("opened stats directory\n");
		}
		//TODAYS DATE AND TIME
		time_t now =time(0);
		tm *ltm = localtime(&now);
		int y,m,d, h, min;
		y=ltm->tm_year-100;
		m = ltm->tm_mon +1;
		d=ltm->tm_mday;
		h= ltm->tm_hour;
		min = ltm->tm_min;
		sprintf(statFile, "%s/stats%02i%02i%02i_%02i%02i.txt",dirName, d,m,y,h,min);
		printf("%s\n", statFile);
		FILE * f = fopen(statFile, "w");
		fclose(f);
	}
}


void Spawner(CoordinateContainer &, CoordinateContainer &); 

int getIteration(){
	return iteration;
}

DeltaPose GetRealVelocity(CoordinateContainer &, CoordinateContainer &);

void controller();

void addIteration(){
	iteration++;
}


void updateAbsPos(b2Vec2 vel){
	absPosition.x += vel.x*timeElapsed;
	absPosition.y += vel.y*timeElapsed;
}

b2Vec2 getAbsPos(){
	return absPosition;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}


void applyController(bool, Task &);


b2Vec2 estimateDisplacementFromWheels();

void reactiveAvoidance(b2World &, Task::simResult &, Task&); //adds two Tasks if crashed but always next up is picked

vertexDescriptor P(vertexDescriptor, CollisionGraph&, Task  , b2World & , std::vector <Leaf> &);

bool backtrackingBuildTree(vertexDescriptor v, CollisionGraph&g, Task s, b2World & w, std::vector <Leaf>&);

bool BFIDBuildTree(vertexDescriptor, CollisionGraph&, Task, b2World &, std::vector <Leaf>&);

Direction getOppositeDirection(Direction d){
    switch (d){
        case Direction::LEFT: return Direction::RIGHT;break;
        case Direction::RIGHT: return Direction::LEFT;break;
		//case Direction::DEFAULT: return Direction::BACK; break;
        default:
        return Direction::DEFAULT;
		break;
    }
}

template <typename V, typename G>
bool isFullLength(V v, const G & g, float length=0){
    if (boost::in_degree(v, g)==0 && length < BOX2DRANGE){
        return false;
    }
    else if (length >=BOX2DRANGE){
        return true;
    }
    else{
        edgeDescriptor inEdge= boost::in_edges(v, g).first.dereference();
        length += g[inEdge].distanceCovered;
		g[v].predecessors++;
        return isFullLength(boost::source(inEdge, g), g, length);
    }

}

void addVertex(vertexDescriptor & src, vertexDescriptor &v1, CollisionGraph &g, Task::Disturbance obs = Task::Disturbance()){
	if (g[src].options.size()>0){
		v1 = boost::add_vertex(g);
		edgeDescriptor e = add_edge(src, v1, g).first;
		g[e].direction =g[src].options[0];
		g[src].options.erase(g[src].options.begin());
		g[v1].totDs=g[src].totDs;
	}
}

void removeIdleNodes(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0);

Sequence getCleanSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

Sequence getUnprocessedSequence(CollisionGraph&, vertexDescriptor, vertexDescriptor root=0); //gets a sequence of summaries of successful tasks, excluding the root node

vertexDescriptor findBestLeaf(CollisionGraph &, std::vector <Leaf>, EndCriteria * refEnd = NULL);
// {
// 	//FIND BEST LEAF
// 	vertexDescriptor best = _leaves[0];
// 	for (vertexDescriptor leaf: _leaves){
// 		if (g[leaf].distanceSoFar>g[best].distanceSoFar){
// 			best = leaf;
// 		}
// 		else if (g[leaf].distanceSoFar==g[best].distanceSoFar){
// 			if (g[leaf].predecessors< g[best].predecessors){ //the fact that this leaf has fewer predecessors implies fewer collisions
// 				best = leaf;
// 			}
// 		}
// 	}
// 	// if (debugOn){
// 	// 	printf("best branch has endpose: x = %f, y= %f, angle = %f\n", g[best].endPose.p.x, g[best].endPose.p.y, g[best].endPose.q.GetAngle());
// 	// }
// 	return best;
// 	//FIND FIRST NODE BEFORE ORIGIN
// 	// std::vector <edgeDescriptor> bestEdges;
// 	// edgeDescriptor e;
// 	// while (best != *(boost::vertices(g).first)){
// 	// 	e = boost::in_edges(best, g).first.dereference();
// 	// 	bestEdges.push_back(e);
// 	// 	best = e.m_source;
// 	// }
// 	// if (!g[0].disturbance.safeForNow){
// 	// printf("plan to go: ");
// 	// for (auto eIt = bestEdges.rbegin(); eIt!=bestEdges.rend(); eIt++){
// 	// 	edgeDescriptor edge = *eIt;
// 	// 	switch (g[edge].direction){
// 	// 		case Direction::DEFAULT: printf("STRAIGHT, "); break;
// 	// 		case Direction::LEFT: printf("LEFT, "); break;
// 	// 		case Direction::RIGHT: printf("RIGHT, "); break;
// 	// 		case Direction::BACK: printf("BACK, "); break;
// 	// 		default: break;

// 	// 	}
// 	// }
// 	// }
// 	// printf("\n");
// 	// return e;
// }

Sequence getPlan(CollisionGraph &, vertexDescriptor);

void printPlan(Sequence);

bool constructWorldRepresentation(b2World & world, Direction d, b2Transform start, Task * curr = NULL){
	//TO DO : calculate field of view: has to have 10 cm on each side of the robot
	bool obStillThere=0;
	const float halfWindowWidth = .1;
	//printf("constructing\n");
	if (d!=LEFT && d!=RIGHT){ //IF THE ROBOT IS NOT TURNING
		std::vector <Point> bounds;
		float qBottomH, qTopH, qBottomP, qTopP, mHead, mPerp;
		float ceilingY, floorY, frontX, backX;
		float boxLength =BOX2DRANGE;		
		Point positionVector, radiusVector, maxFromStart; 
		if(d==BACK){
			float x = start.p.x - (BACK_DISTANCE+ ROBOT_HALFLENGTH)* cos(start.q.GetAngle());
			float y = start.p.y - (BACK_DISTANCE+ROBOT_HALFLENGTH)* sin(start.q.GetAngle());
			start = b2Transform(b2Vec2(x, y), b2Rot(start.q.GetAngle()));
			//boxLength += BACK_DISTANCE+ROBOT_HALFLENGTH;
			//printf("modified boxlength = %f, start x = %f, y= %f\n", boxLength, start.p.x, start.p.y);
		}
		radiusVector.polarInit(boxLength, start.q.GetAngle());
		//printf("radius vector = x=%f, y=%f\n", radiusVector.x, radiusVector.y);
		maxFromStart = Point(start.p) + radiusVector;
		//printf("max from start length = %f\n", maxFromStart.r);
		//FIND THE BOUNDS OF THE BOX
		b2Vec2 unitPerpR(-sin(start.q.GetAngle()), cos(start.q.GetAngle()));
		b2Vec2 unitPerpL(sin(start.q.GetAngle()), -cos(start.q.GetAngle()));
		bounds.push_back(Point(start.p)+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth));
		bounds.push_back(Point(start.p)+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
		bounds.push_back(maxFromStart+Point(unitPerpL.x *halfWindowWidth, unitPerpL.y*halfWindowWidth)); 
		bounds.push_back(maxFromStart+Point(unitPerpR.x *halfWindowWidth, unitPerpR.y*halfWindowWidth));
		Point top, bottom;
		comparator compPoint;
		std::sort(bounds.begin(), bounds.end(), compPoint); //sort bottom to top
		bottom = bounds[0];
		top = bounds[3];
		if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle())!=0){
			//FIND PARAMETERS OF THE LINES CONNECTING THE a
			mHead = sin(start.q.GetAngle())/cos(start.q.GetAngle()); //slope of heading direction
			mPerp = -1/mHead;
			qBottomH = bottom.y - mHead*bottom.x;
			qTopH = top.y - mHead*top.x;
			qBottomP = bottom.y -mPerp*bottom.x;
			qTopP = top.y - mPerp*top.x;
		}
		else{
			ceilingY = std::max(top.y, bottom.y); 
			floorY = std::min(top.y, bottom.y); 
			frontX = std::min(top.x, bottom.x);
			backX = std::max(top.x, bottom.x);
		}
		//CREATE POINTS
		for (auto p:currentBox2D){
			bool include;
			if (sin(start.q.GetAngle())!=0 && cos(start.q.GetAngle()!=0)){
				ceilingY = mHead*p.x +qTopH;
				floorY = mHead*p.x+qBottomH;
				float frontY= mPerp*p.x+qBottomP;
				float backY = mPerp*p.x+qTopP;
				//include = (p!= *(&p-1)&& p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY);
				include = p.y >=floorY && p.y<=ceilingY && p.y >=frontY && p.y<=backY;
			}
			else{
				include = (p.y >=floorY && p.y<=ceilingY && p.x >=frontX && p.x<=backX);
			}
			if (include){
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (curr !=NULL){ //
					if (curr->disturbance.isValid()){
						if (p.isInRadius(currentTask.disturbance.getPosition())){
							obStillThere =1;
						}
					}
				}
				bodyDef.position.Set(p.x, p.y); 
				bodies++;
				body = world.CreateBody(&bodyDef);
				body->CreateFixture(&fixtureDef);
			}
		}
		}
		else{ //IF DIRECTION IS LEFT OR RIGHT 
		for (auto p:currentBox2D){
			if (p.isInRadius(start.p, ROBOT_HALFLENGTH -ROBOT_BOX_OFFSET_X + 0.01)){ //y range less than 20 cm only to ensure that robot can pass + account for error
				b2Body * body;
				b2BodyDef bodyDef;
				b2FixtureDef fixtureDef;
				bodyDef.type = b2_dynamicBody;
				b2PolygonShape fixture; //giving the point the shape of a box
				fixtureDef.shape = &fixture;
				fixture.SetAsBox(.001f, .001f); 
				if (curr !=NULL){ //
					if (curr->disturbance.isValid()){
						if (p.isInRadius(currentTask.disturbance.getPosition())){
							obStillThere =1;
						}
					}
				}
				bodyDef.position.Set(p.x, p.y); 
				body = world.CreateBody(&bodyDef);
				bodies++;
				body->CreateFixture(&fixtureDef);
			}
		// 	else if (curr!=NULL){
		// 		if (curr->disturbance.isValid()){
		// 			if (p.isInRadius(currentTask.disturbance.getPosition())){
		// 				obStillThere =1;
		// 				b2Body * body;
		// 				b2BodyDef bodyDef;
		// 				b2FixtureDef fixtureDef;
		// 				bodyDef.type = b2_dynamicBody;
		// 				b2PolygonShape fixture; //giving the point the shape of a box
		// 				fixtureDef.shape = &fixture;
		// 				fixture.SetAsBox(.001f, .001f); 
		// 				bodyDef.position.Set(p.x, p.y); 
		// 				body = world.CreateBody(&bodyDef);
		// 				bodies++;
		// 				body->CreateFixture(&fixtureDef);
		// 			}
		// 		}
		// }
		}
		

	}
	return obStillThere;
}

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void makeBody(b2World &, b2Vec2, int); //takes world, position and body count

void applyTransitionMatrix3M(CollisionGraph&, vertexDescriptor, Direction); //DEFAULT, LEFT, RIGHT

void applyTransitionMatrix4M(CollisionGraph&, vertexDescriptor, Direction); //DEFAULT, LEFT, RIGHT, BACK

bool betterThanLeaves(CollisionGraph&, vertexDescriptor, std::vector <Leaf>, EndedResult &);

void backtrack(CollisionGraph&, vertexDescriptor&);

};




 #endif