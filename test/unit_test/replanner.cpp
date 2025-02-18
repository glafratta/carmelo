#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=1;
    Disturbance target1;
    vertexDescriptor solution=TransitionSystem::null_vertex();
    if (argc>2){
        solution=vertexDescriptor(atoi(argv[2]));
    }
    else{
        throw;
    }
    if (argc>3){
        if (atoi(argv[3])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
            printf("target");
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.27;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    conf.setBenchmarking(true, "simulation_benchmarking");
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.Spawner();
    int n_v=conf.transitionSystem.m_vertices.size();
    auto og_plan=conf.planVertices;
    if (argv[1]=="empty"){
        og_plan={2};
    }
    conf.printPlan(&conf.planVertices);
    conf.addIteration();
    int og_step=0;
    conf.changeTask(1, og_step, conf.planVertices);
    conf.getTask()->motorStep=0;
    conf.currentVertex=*(conf.planVertices.end()-1);
    vertexDescriptor prev=*(conf.planVertices.end()-2);
    conf.currentEdge=boost::edge(prev, conf.currentVertex, conf.transitionSystem).first;
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    math::applyAffineTrans(shift, conf.transitionSystem);    
    if (argc>4){
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    if (argc > 5){
        conf.controlGoal.disturbance.bf.pose = -shift;
        conf.controlGoal.disturbance.bf.pose.q.Set(M_PI);
        printf("back");
        og_plan={3, 5, 2};
        n_v+=7;
    }
    conf.planVertices.clear();
    conf.Spawner();
    conf.printPlan(&conf.planVertices);
    if (conf.transitionSystem.m_vertices.size() > n_v){
        printf("size error = %i\n", conf.transitionSystem.m_vertices.size()-n_v);
        return 2;
    }
    if (og_plan!=conf.planVertices){
        printf("wrong plan\n");
        return 1;
    }
     
    return 0;
}