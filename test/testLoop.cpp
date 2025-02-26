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
       // c->getTask()->correct(action, c->getTask()->motorStep);
        EndedResult er = c->controlGoal.checkEnded();
	    if (er.ended){
            Disturbance new_goal(PURSUE, c->controlGoal.start.p, c->controlGoal.start.q.GetAngle());
		    c->controlGoal = Task(new_goal, UNDEFINED);
            printf("goal reached\n");
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
	Disturbance target(PURSUE, b2Vec2(BOX2DRANGE, 0));    
    Task controlGoal;
    bool debug=1;
    Configurator conf(controlGoal, debug);
    conf.simulationStep=0.5;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    AltCallback cb(&conf);
    ci.data2fp.emplace(Pointf(-1.0, -1.0));
    conf.data2fp.emplace(Pointf(-1.0, -1.0));
    conf.Spawner();
    //cb.step();
   // conf.planVertices =conf.changeTask(1, cb.ogStep, conf.planVertices);
    do{
        cb.step();
    }while (!conf.planVertices.empty() || conf.getTask()->motorStep>0);    
  //  conf.getTask()->motorStep=0;
    conf.Spawner();
    printGraph(conf.transitionSystem);

}