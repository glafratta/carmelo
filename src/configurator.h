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
	CoordinateContainer data2fp;
	bool ready=0;
	bool newData=0;
	bool stop=0;

	void setReady(bool b);

	bool isReady();


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

void setBenchmarking(bool , char * , char * _dir=NULL);

bool is_benchmarking(){
	return benchmark;
}

bool Spawner(); 

int getIteration(){
	return iteration;
}

void addIteration(int i=1){
	iteration+=i;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}

void dummy_vertex(vertexDescriptor src);

class Explorer{
	Disturbance getDisturbance(TransitionSystem&, const vertexDescriptor&, b2World &, const Direction &, const b2Transform&);

	Task task_to_execute(const TransitionSystem &, const edgeDescriptor&);

	simResult simulate(Task, b2World &, float _simulationStep=BOX2DRANGE);

	void backtrack(std::vector <vertexDescriptor>&, std::vector <vertexDescriptor>&, const std::set<vertexDescriptor>&, TransitionSystem&, std::vector <vertexDescriptor>&);

	std::vector <vertexDescriptor> splitTask(vertexDescriptor v, TransitionSystem&, Direction, vertexDescriptor src=TransitionSystem::null_vertex());

	void propagateD(vertexDescriptor, vertexDescriptor, TransitionSystem&, std::vector<vertexDescriptor>*propagated=NULL, std::set<vertexDescriptor>*closed=NULL, StateMatcher::MATCH_TYPE match=StateMatcher::_FALSE);

	void pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>>, TransitionSystem&, vertexDescriptor&, vertexDescriptor&,std::vector <vertexDescriptor>&, std::vector<std::pair<vertexDescriptor, vertexDescriptor>>&); //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end

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

std::vector<vertexDescriptor> explorer(vertexDescriptor, TransitionSystem&, Task, b2World &); //evaluates only after DEFAULT, internal one step lookahead

void skip_reduced(edgeDescriptor &, TransitionSystem &, const std::vector<vertexDescriptor> &, std::vector<vertexDescriptor>::iterator);

EndedResult estimateCost(State&, b2Transform, Direction); //returns whether the controlGoal has ended and fills node with cost and error

float evaluationFunction(EndedResult);

void unexplored_transitions(TransitionSystem&, const vertexDescriptor&);

void transitionMatrix(State&, Direction, vertexDescriptor); //DEFAULT, LEFT, RIGHT

void applyTransitionMatrix(TransitionSystem&, vertexDescriptor, Direction,bool, vertexDescriptor, std::vector<vertexDescriptor>&);

void addToPriorityQueue(vertexDescriptor, std::vector <vertexDescriptor>&, TransitionSystem&, const std::set<vertexDescriptor>&);

void addToPriorityQueue(Frontier, std::vector <Frontier>&, TransitionSystem&, vertexDescriptor goal=TransitionSystem::null_vertex());

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(State, TransitionSystem&, State * src, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, std::vector <vertexDescriptor>* others=NULL, bool relax=0, bool wholeTask=false); //matches to most likely

float approximate_angle(const float &, const Direction &, const simResult::resultType &);

void shift_states(TransitionSystem &, const std::vector<vertexDescriptor>&, const b2Transform &); //shifts a sequence of states by a certain transform


};

class Planner{
	void planPriority(TransitionSystem&, vertexDescriptor); 

	std::vector <Frontier> frontierVertices(vertexDescriptor, TransitionSystem&, Direction , bool been=0); //returns the closest vertices to the start vertex which are reached by executing a task of the specified direction

	std::vector <vertexDescriptor> planner(TransitionSystem&, vertexDescriptor, vertexDescriptor goal=TransitionSystem::null_vertex(), bool been=0, const Task* custom_ctrl_goal=NULL, bool * finished =NULL) ;

};

//float taskRotationError(); // returns lateral displacement error (local y)

//inputs: g, src vertex, b2d world, direction of the task to be created

void trackDisturbance(b2Transform &, Task::Action, float); //open loop

void updateGraph(TransitionSystem&, const b2Transform &);


void adjust_rw_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);

//std::vector <edgeDescriptor> inEdgesRecursive(vertexDescriptor, TransitionSystem&, Direction ); //returns a vector of all in-edges leading to the vertex which have the same direction (most proximal first)

//std::vector <edgeDescriptor> frontierVertices(vertexDescriptor, TransitionSystem&, Direction , bool been=0); //returns the closest vertices to the start vertex which are reached by executing a task of the specified direction


//void recall_plan_from(const vertexDescriptor&, TransitionSystem & , b2World &, std::vector <vertexDescriptor>&, bool&, Disturbance *dist);

//std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor>, TransitionSystem&);


//std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(vertexDescriptor, TransitionSystem&, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, std::vector <vertexDescriptor>* others=NULL); //has a safety to prevent matching a vertex with self

//void changeStart(b2Transform&, vertexDescriptor, TransitionSystem&, const b2Transform& shift=b2Transform_zero); //if task at vertex v fails, start is set to v's predecessor's end

//void match_setup(bool&, StateMatcher::MATCH_TYPE&, const vertexDescriptor &, std::vector<vertexDescriptor>&, const Direction&, TransitionSystem &);



std::pair <bool, Direction> getOppositeDirection(Direction);

void resetPhi(TransitionSystem&g);

void printPlan(std::vector <vertexDescriptor>* p=NULL);


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





void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(ConfiguratorInterface *);

static void run(Configurator *);

void trackTaskExecution(Task &);

std::vector <vertexDescriptor> changeTask(bool, int&, std::vector <vertexDescriptor>);

int motorStep(Task::Action a);

void setSimulationStep(float f){
	simulationStep=f;
}

//void done_that(vertexDescriptor&, bool &, b2World &, std::vector <vertexDescriptor>&);

//bool current_task_equivalent(const Task &,const  Task &, const vertexDescriptor&);
void ts_cleanup(TransitionSystem *);

};




 #endif