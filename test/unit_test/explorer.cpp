#include "../callbacks.h"



int main(int argc, char** argv){
    
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
    bool debug=1;
    Disturbance target1;
    float simulationStep=0.5;
    if (argc>2){
        if (atoi(argv[2])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);   
            simulationStep=std::max(ROBOT_HALFWIDTH*2, ROBOT_HALFLENGTH*2);
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=simulationStep;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    conf.planVertices =conf.explorer(conf.currentVertex, conf.transitionSystem, *conf.getTask(), world);
    if (!conf.planVertices.empty()){
        return 1;
    }
    debug::print_graph(conf.transitionSystem, target1, std::vector<vertexDescriptor>(), conf.currentVertex );
    return 0;
}