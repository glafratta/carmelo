#ifndef CALLBACKS_H
#define CALLBACKS_H
#include "CppTimer.h"
#include <thread>
#include "test_essentials.h"

class DataInterface {
public:
int iteration = 0;
ConfiguratorInterface * ci;

char * folder;
    DataInterface(ConfiguratorInterface * _ci): ci(_ci){}

	bool newScanAvail(){ //uncomment sections to write x and y to files		
        iteration++;
    	ci->ready=0;
		ci->data2fp.clear();
		char filePath[256];
        char folderName[256];
        sprintf(folderName,"%s", folder);
		sprintf(filePath, "%smap%04d.dat", folderName, iteration);
        FILE *f;
        if (!(f=fopen(filePath, "r"))){
            if (iteration>1){
                iteration=1;
            }
            else{
                ci->stop=1;
                return false;
            }
        }
        else {
            fclose(f);
        }
		std::ifstream file(filePath);
        float x2, y2;
		while (file>>x2>>y2){
            x2 = round(x2*100)/100;
			y2 = round(y2*100)/100;
            Pointf  p2(x2,y2);
            ci->data2fp.insert(p2);
		}
		file.close();
        ci->setReady(1);
        ci->iteration++;
        return true;
		


	}
};

class StepCallback{
    float L=0,R=0;
    Configurator * c;
    int ogStep=0;
public:

    StepCallback()=default;

    StepCallback(Configurator * _c): c(_c){}
    void step(){
        if (!c->running ){
            return;
        }
        ExecutionError ee =c->trackTaskExecution(*c->getTask());
        Task::Action action= c->getTask()->getAction();
        EndedResult er = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, true);//true
        if (er.ended &( c->getTask()->motorStep<1 & c->transitionSystem[c->movingEdge].direction!=STOP && c->planVertices.empty() && c->getIteration()>1)){ //& c->getTask()->motorStep<1

            Disturbance new_goal(PURSUE, c->controlGoal.start.p, c->controlGoal.start.q.GetAngle());
		    c->controlGoal = Task(new_goal, UNDEFINED);
            b2Vec2 v = c->controlGoal.disturbance.getPosition() - b2Vec2(0,0);
        	FILE * f = fopen(c->statFile, "a+");
            fprintf(f, "!");
            fclose(f);

	    }
	    c->planVertices =c->changeTask(c->getTask()->change,  ogStep, c->planVertices);
        L=c->getTask()->getAction().getLWheelSpeed();
        R= c->getTask()->getAction().getRWheelSpeed();
    }
};


 #endif