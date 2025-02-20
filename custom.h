#include "configurator.h"
#include "a1lidarrpi.h"
#include "alphabot.h"
#include <stdio.h>
#include <stdlib.h>
#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#define _USE_MATH_DEFINES

void get_Foldername(char* custom, char name[60]){
    time_t now =time(0);
	tm *ltm = localtime(&now);
	int y,m,d, h, min;
	y=ltm->tm_year-100;
	m = ltm->tm_mon +1;
	d=ltm->tm_mday;
	h= ltm->tm_hour;
	min = ltm->tm_min;
	sprintf(name, "%s_%02i%02i%02i_%02i%02i",custom, d,m,y,h,min);
}

//void forget(Configurator*);

//Disturbance set_target(int&, b2Transform);




class LidarInterface : public A1Lidar::DataInterface{
ConfiguratorInterface * ci;
public: 
    int mapCount =0;

    LidarInterface(ConfiguratorInterface * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (ci == NULL){
			printf("null pointer to ci\n");
			return;
		}
		ci->data2fp.clear();
		mapCount++;
		Pointf p, p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		if (ci->debugOn){
			f=fopen(name, "w");
		}
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x2 = round(data.x*100)/100;
				float y2 = round(data.y*100)/100;
				p2f=Pointf(x2, y2);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		if (ci->debugOn){
		fclose(f);
		}
		if (!ci->data2fp.empty()){
			ci->setReady(1);
		}
		ci->iteration++;

	}


};

class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    float L=0;
	float R=0;
public:
int ogStep=0;
Configurator * c;
int run=0;

MotorCallback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	if (c->getIteration() <=0){
		return;
	}
	if (!c->running){
		motors.setRightWheelSpeed(0);
 	   motors.setLeftWheelSpeed(0);		
	}

	c->trackTaskExecution(*c->getTask());
	EndedResult er = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, false);
	if (er.ended ){ 
		c->stop();

	}
	c->planVertices = c->changeTask(c->getTask()->change,  ogStep, c->planVertices);
	R= c->getTask()->getAction().getRWheelSpeed();
	L=c->getTask()->getAction().getLWheelSpeed(); //*1.05
	if (c->getTask()->direction==LEFT){
		R*=1.37; 
		L*=1.37;
	}
	else if (c->getTask()->direction==RIGHT){
		R*=1.07; //17
		L*=1.07;
	}
	else if (c->getTask()->direction==DEFAULT){
		R*=1.15*1.1;
		L*=1.15;
	}
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
}
};
