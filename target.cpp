#include "custom.h"

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	ConfiguratorInterface configuratorInterface;
    Configurator configurator(controlGoal);
	configurator.planning =1;
	if (argc>2){
		configurator.planning= atoi(argv[2]);
	}
	if (configurator.planning){
		configurator.setBenchmarking(1, "s4_rt_newbodies");
	}
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface);
	MotorCallback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
