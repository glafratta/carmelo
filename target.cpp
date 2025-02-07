#include "custom.h"

void forget(Configurator *c){}

// void Configurator::done_that(vertexDescriptor& src, bool & plan_works, b2World & world, std::vector<vertexDescriptor>& plan_provisional){
// 		//std::pair<bool, vertexDescriptor> been(false, TransitionSystem::null_vertex());
// 		//was ve instead of src
// 		std::vector <vertexDescriptor> options_src;
// 		bool fin=controlGoal.checkEnded(transitionSystem[src], UNDEFINED, true).ended;
// 		printf("finished=%i, plan=%i, step=%i\n", fin, !planVertices.empty(), currentTask.motorStep);
// 		if ( !fin && planVertices.empty()&& currentTask.motorStep==0){
// 		//printf("is target=%i, task ended = %i\n", target.getAffIndex()==PURSUE, fin);
// 			printf("NOW CHECKING FOR MATCHES\n");
// 			//bool using_kmeans=1;
// 			std::vector <BodyFeatures> b_features=worldBuilder.getFeatures(data2fp, transitionSystem[movingVertex].endPose, 
// 												currentTask.direction, BOX2DRANGE, 0.15, WorldBuilder::CLUSTERING::PARTITION);
// 			State s_temp;
// 			if (b_features.size()>0){
// 				WorldBuilder::CompareCluster compareCluster;
// 				BodyFeatures closest_D= *(std::min_element(b_features.begin(), b_features.end(), compareCluster));
// 				s_temp.Dn= Disturbance(closest_D); //assumes 1 item length
// 				printf("d to match: ");
// 				debug::print_pose(s_temp.Dn.pose());
// 				bool closest_match=true;
// 				findMatch(s_temp,transitionSystem, NULL, UNDEFINED, StateMatcher::DISTURBANCE, &options_src, closest_match);
// 				printf("looked for matches, potential: %i\n", options_src.size());
// 				//FIND STATE WHICH matches the relationship with the disturbance
// 			}
// 			else{
// 				vertexDescriptor new_v;
// 				auto e=addVertex(currentVertex, new_v, transitionSystem);
// 				simResult fake_result;
// 				fake_result.endPose=b2Transform(b2Vec2(BOX2DRANGE, 0), b2Rot(0));
// 				fake_result.step=100;
// //				printf("made fake result\n");
// 				gt::fill(fake_result, transitionSystem[new_v].ID);
// //				printf("filled\n");
// 				transitionSystem[e.first].direction=DEFAULT;
// 				transitionSystem[e.first].it_observed=iteration;
// 				//transitionSystem[e.first].step=100;
// 				plan_provisional={new_v};
// 				plan_works=true;
// 			}
// 			//been= been_there(transitionSystem, where); 
// 		}
// 		else{
// 			//plan_works=true;//temporary to simplify things
// 			plan_works=checkPlan(world,planVertices,transitionSystem, s_temp.Dn, transitionSystem[movingVertex].start, currentVertex);
// 		}
// 		for (auto o:options_src){
// 			printf("recall from %i : ", o);
// 			debug::print_pose(transitionSystem[o].Dn.pose());
// 			printf("\n");
// 			recall_plan_from(o, transitionSystem, world, plan_provisional, plan_works);
// 			if (plan_works){
// 				break;
// 			}
// 		}


// }


Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	if (run%2!=0 && run >0){
		result= Disturbance(PURSUE, b2Vec2(1.0f, 0.0f), 0.0f);
	}
	else{
		result= Disturbance(PURSUE, b2Vec2(-1.0f, 0.0f), 0.0f);
	}
	run++;
	return result;
}

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
		configurator.setBenchmarking(1, "forward-recycle");
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
	CameraCallback cameraCB(&cb);
	Libcam2OpenCV camera;
    camera.registerCallback(&cameraCB);
    Libcam2OpenCVSettings settings;
    settings.framerate = FPS;
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
    camera.start(settings);
	motors.start();
	getchar();
	configurator.stop();
	motors.stop();
	lidar.stop();
	camera.stop();
}
	
	
