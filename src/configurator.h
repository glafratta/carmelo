#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
//#include "worldbuilder.h"
#include <algorithm>
#include <sys/stat.h>
#include "debug.h"

//FOR DEBUG
//

const std::map<Direction, char*> dirmap={{DEFAULT, "DEFAULT"}, {LEFT, "LEFT"}, {RIGHT, "RIGHT"}, {STOP, "STOP"}, {UNDEFINED, "UNDEFINED"}, {BACK, "BACK"}};

//typedef b2Transform DeltaPose;

class ConfiguratorInterface{
public:
	bool debugOn=0;
	int iteration=0;
	//CoordinateContainer data;
	CoordinateContainer data2fp;
	//cv::Mat visual_field;
	bool ready=0;
	bool newData=0;
	//PointCloudProc * pcProc=NULL;
	bool stop=0;

	void setReady(bool b);

	bool isReady();

	// void updatePCProc(){
	// 	pcProc->updatePrevious(data);
	// }



};

class Configurator{
protected:
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	Task currentTask;
	bool benchmark=0;
public:
	ConfiguratorInterface * ci=NULL;
	bool running =0;
	std::thread * t=NULL;
	bool debugOn=0;
	float simulationStep=2*std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH);
	b2Transform ogGoal;
	Task controlGoal;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	float timeElapsed =0;
	CoordinateContainer data2fp;
	bool planning =1;
	char statFile[100];
	char bodyFile[100];
	bool timerOff=0;
	int bodies=0;
	std::vector <vertexDescriptor> planVertices;
	TransitionSystem transitionSystem;
	StateMatcher matcher;
	WorldBuilder worldBuilder;
	vertexDescriptor movingVertex;
	vertexDescriptor currentVertex;
	edgeDescriptor movingEdge, currentEdge;
	std::unordered_map <State*, ExecutionError> errorMap;

Configurator()=default;

Configurator(Task _task, bool debug =0, bool noTimer=0): controlGoal(_task), currentTask(_task), debugOn(debug), timerOff(noTimer){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	worldBuilder.debug=debug;
	ogGoal=controlGoal.disturbance.pose();
	movingVertex=boost::add_vertex(transitionSystem);
	transitionSystem[movingVertex].Di=controlGoal.disturbance;
	currentVertex=movingVertex;
	currentTask.action.setVelocities(0,0);
	gt::fill(simResult(), &transitionSystem[movingVertex]);
}

void setBenchmarking(bool b, char * new_folder){
	benchmark =b;
		if (benchmark){
		char dirName[50];
		sprintf(dirName, "benchmark");
		if (!opendir(dirName)){
			mkdir(dirName, 0777);
		}
		char new_path[60];
		sprintf(new_path, "%s/%s", dirName, new_folder);
		printf("%s", new_path);
		if (!opendir(new_path)){
			mkdir(new_path, 0777); //""
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
		sprintf(statFile, "%s/stats%02i%02i%02i_%02i%02i.txt",new_path, d,m,y,h,min);
		printf("%s\n", statFile);
		FILE * f = fopen(statFile, "w");
		printf("open\n");
		fclose(f);
	}
	printf("set\n");
}

bool is_benchmarking(){
	return benchmark;
}

bool Spawner(); 

int getIteration(){
	return iteration;
}

void addIteration(){
	iteration++;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}

void dummy_vertex(vertexDescriptor src);

float taskRotationError(); // returns lateral displacement error (local y)

//inputs: g, src vertex, b2d world, direction of the task to be created
Disturbance getDisturbance(TransitionSystem&, const vertexDescriptor&, b2World &, const Direction &);

simResult simulate(State&, State, Task, b2World &, float _simulationStep=BOX2DRANGE);

void backtrack(std::vector <vertexDescriptor>&, std::vector <vertexDescriptor>&, const std::set<vertexDescriptor>&, TransitionSystem&);

std::vector <vertexDescriptor> splitTask(vertexDescriptor v, TransitionSystem&, Direction, vertexDescriptor src=TransitionSystem::null_vertex());

std::vector<std::pair<vertexDescriptor, vertexDescriptor>> propagateD(vertexDescriptor, vertexDescriptor, TransitionSystem&, std::vector<vertexDescriptor>*propagated=NULL, std::set<vertexDescriptor>*closed=NULL);

void pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>>, TransitionSystem&, vertexDescriptor&, vertexDescriptor&,std::vector <vertexDescriptor>&, std::vector<std::pair<vertexDescriptor, vertexDescriptor>>&); //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end

void clearFromMap(std::vector<std::pair<vertexDescriptor, vertexDescriptor>>, TransitionSystem&, std::unordered_map<State*, ExecutionError>);

void trackDisturbance(b2Transform &, Task::Action, float);

void updateGraph(TransitionSystem&, ExecutionError error=ExecutionError());

void planPriority(TransitionSystem&, vertexDescriptor); 

void adjustStepDistance(vertexDescriptor, TransitionSystem &, Task*, float&, std::pair<bool,vertexDescriptor> tgt=std::pair(false,TransitionSystem::null_vertex()));

std::vector <edgeDescriptor> inEdgesRecursive(vertexDescriptor, TransitionSystem&, Direction ); //returns a vector of all in-edges leading to the vertex which have the same direction (most proximal first)

std::vector <Frontier> frontierVertices(vertexDescriptor, TransitionSystem&, Direction , bool been=0); //returns the closest vertices to the start vertex which are reached by executing a task of the specified direction

//void recall_plan_from(const vertexDescriptor&, TransitionSystem & , b2World &, std::vector <vertexDescriptor>&, bool&);

std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor>, TransitionSystem&);

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(State, TransitionSystem&, State * src, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, std::vector <vertexDescriptor>* others=NULL, bool relax=0); //matches to most likely

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(vertexDescriptor, TransitionSystem&, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, std::vector <vertexDescriptor>* others=NULL); //has a safety to prevent matching a vertex with self

void changeStart(b2Transform&, vertexDescriptor, TransitionSystem&); //if task at vertex v fails, start is set to v's predecessor's end

std::vector<std::pair<vertexDescriptor, vertexDescriptor>> explorer(vertexDescriptor, TransitionSystem&, Task, b2World &); //evaluates only after DEFAULT, internal one step lookahead

std::pair <bool, Direction> getOppositeDirection(Direction);

void resetPhi(TransitionSystem&g);

void printPlan(std::vector <vertexDescriptor>* p=NULL);

void applyAffineTrans(const b2Transform& , Task& );

void applyAffineTrans(const b2Transform&, TransitionSystem&);

void applyAffineTrans(const b2Transform&, Disturbance&);

std::pair<edgeDescriptor, bool> addVertex(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Edge edge=Edge(), bool topDown=0){ //returns edge added
	std::pair<edgeDescriptor, bool> result;
	result.second=false;
	if (g[src].options.size()>0 || topDown){
		v1 = boost::add_vertex(g);
		result = add_edge(src, v1, g);
		g[result.first] =edge;
		g[result.first].direction=g[src].options[0];
		g[result.first].it_observed=iteration;
		if (!topDown){
			g[src].options.erase(g[src].options.begin());
		}

	}
	return result;
}

std::pair <edgeDescriptor, bool> add_vertex_now(vertexDescriptor &, vertexDescriptor &, TransitionSystem &, Disturbance,Edge edge=Edge(), bool topDown=0);

std::pair <edgeDescriptor, bool> add_vertex_retro(vertexDescriptor &, vertexDescriptor &, TransitionSystem &, Disturbance,Edge edge=Edge(), bool topDown=0);


void setStateLabel(State& s, vertexDescriptor src, Direction d){
	if(d!=currentTask.direction & src==movingVertex){
		if ( d!=DEFAULT){
			s.label=VERTEX_LABEL::ESCAPE;
		}	
	}		
	else if (transitionSystem[src].label==ESCAPE &d==DEFAULT){ //not two defaults
		s.label=ESCAPE2;
	}

}

std::vector <vertexDescriptor> planner(TransitionSystem&, vertexDescriptor, vertexDescriptor goal=TransitionSystem::null_vertex(), bool been=0, const Task* custom_ctrl_goal=NULL, bool * finished =NULL) ;

EndedResult estimateCost(State&, b2Transform, Direction); //returns whether the controlGoal has ended and fills node with cost and error

float evaluationFunction(EndedResult);

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void unexplored_transitions(TransitionSystem&, const vertexDescriptor&);

void transitionMatrix(State&, Direction, vertexDescriptor); //DEFAULT, LEFT, RIGHT

void applyTransitionMatrix(TransitionSystem&, vertexDescriptor, Direction,bool, vertexDescriptor);

void addToPriorityQueue(vertexDescriptor, std::vector <vertexDescriptor>&, TransitionSystem&, const std::set<vertexDescriptor>&);

void addToPriorityQueue(Frontier, std::vector <Frontier>&, TransitionSystem&, vertexDescriptor goal=TransitionSystem::null_vertex());
																		//and straight lines
std::pair <bool, vertexDescriptor> been_there(TransitionSystem &, Disturbance target=Disturbance());



ExecutionError trackTaskExecution(Task &);

b2Transform assignDeltaPose(Task::Action, float);

std::vector <vertexDescriptor> changeTask(bool, int&, std::vector <vertexDescriptor>);

int motorStep(Task::Action a);

void setSimulationStep(float f){
	simulationStep=f;
}

void done_that(vertexDescriptor&, bool &, b2World &, std::vector <vertexDescriptor>&);



};




 #endif