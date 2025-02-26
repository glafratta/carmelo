#include "callbacks.h"

class AltCallback{
    float L=0,R=0;
    Configurator * c;

    vertexDescriptor start;
public:
        int ogStep=0;
        
    AltCallback(Configurator * _c): c(_c){
        start=c->movingEdge.m_target;
    }
    void step(){
        c->trackTaskExecution(*c->getTask());
        Task::Action action= c->getTask()->getAction();
        c->getTask()->correct(action, c->getTask()->motorStep);
        EndedResult er = c->controlGoal.checkEnded();
	    if (er.ended){
            Disturbance new_goal(PURSUE, c->transitionSystem[start].endPose.p);
		    c->controlGoal = Task(new_goal, UNDEFINED);
            printf("goal reached\n");
	    }
        if (er.ended || start==c->movingVertex){
		    start=c->currentVertex;
	    }
	    c->planVertices =c->changeTask(c->getTask()->change,  ogStep, c->planVertices);
        L=c->getTask()->getAction().getLWheelSpeed();
        R= c->getTask()->getAction().getRWheelSpeed();
        printf("L=%f, R=%f\n", L, R);
    }
};




Disturbance set_target(int& run, b2Transform start){

}

void forget(Configurator* c){}


int main(int argc, char** argv){
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
    bool debug=0;
    Disturbance target1(PURSUE, b2Vec2(0, -0.5), -M_PI_2);    
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.5;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    AltCallback cb(&conf);
    ci.data2fp.emplace(Pointf(-1.0, -1.0));
    conf.data2fp.emplace(Pointf(-1.0, -1.0));
    conf.Spawner();
   // cb.step();
    auto v1 = boost::add_vertex(conf.transitionSystem);
    auto e1 = boost::add_edge(conf.planVertices[0], v1, conf.transitionSystem);
    conf.transitionSystem[e1.first].direction=STOP;
    conf.transitionSystem[e1.first].step=0;
    conf.planVertices.push_back(v1);
    do {
    cb.step();
    }while(conf.getTask()->direction!=STOP & conf.getTask()->motorStep>=0);
    debug::graph_file(2, conf.transitionSystem, conf.controlGoal.disturbance, conf.planVertices, conf.currentVertex);
    Disturbance target2(PURSUE, conf.controlGoal.start.p, conf.controlGoal.start.q.GetAngle());    
    conf.controlGoal= Task(target2, DEFAULT);
    conf.Spawner();
    auto v2 = boost::add_vertex(conf.transitionSystem);
    auto e2 = boost::add_edge(4, v2, conf.transitionSystem);
    conf.transitionSystem[e2.first].direction=STOP;
    conf.transitionSystem[e2.first].step=0;
    conf.planVertices.push_back(v2);
    do {
    cb.step();
    }while(conf.getTask()->direction!=STOP & conf.getTask()->motorStep>=0);
    debug::graph_file(3, conf.transitionSystem, conf.controlGoal.disturbance, conf.planVertices, conf.currentVertex);


}