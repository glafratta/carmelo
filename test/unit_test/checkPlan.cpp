#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=0;
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
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.27;
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
    conf.explorer(conf.currentVertex, conf.transitionSystem, *conf.getTask(), world);
    conf.ts_cleanup(&conf.transitionSystem);		

    
    std::vector <vertexDescriptor> plan=conf.planner(conf.transitionSystem, conf.currentVertex);
    conf.planVertices=plan;
    int n_v=conf.transitionSystem.m_vertices.size();
    conf.printPlan(&plan);
    int og=0;
    conf.changeTask(true, og, conf.planVertices);
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    int steps= atoi(argv[4]);
    //float distanceTraversed = MOTOR_CALLBACK*conf.getTask()->action.getLinearSpeed()*(steps-conf.getIteration());
    // b2Transform shift;
    // shift.q.Set(MOTOR_CALLBACK*conf.getTask()->action.getOmega()*steps);
    // shift.p.x= cos(shift.q.GetAngle())*distanceTraversed;
    // shift.p.y= sin(shift.q.GetAngle())*distanceTraversed;
    // math::applyAffineTrans(-shift, conf.transitionSystem);    
    // math::applyAffineTrans(-shift, conf.controlGoal.disturbance);
    int ogstep=conf.transitionSystem[conf.currentEdge].step;
    for (int i=0;i<di.iteration*2; i++){
        conf.trackTaskExecution(*conf.getTask());
        conf.changeTask(conf.getTask()->change, ogstep, conf.planVertices);
    }
    if (argc>4){
        di.iteration=steps;
        conf.addIteration(steps-conf.getIteration()+1);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    conf.debugOn=1;
  //  bool plan_works= conf.checkPlan(world,conf.planVertices, conf.transitionSystem, conf.transitionSystem[solution].Dn, conf.transitionSystem[conf.movingVertex].start);
    

   // boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.resetPhi(conf.transitionSystem);
    //int og_step=0;
    //conf.changeTask(true, og_step, conf.planVertices);
    conf.explorer(conf.movingVertex, conf.transitionSystem, *conf.getTask(), world);
    conf.ts_cleanup(&conf.transitionSystem);
    bool finished=false;	
    plan=conf.planner(conf.transitionSystem, conf.currentVertex,TransitionSystem::null_vertex(), false, NULL, &finished);
    conf.planVertices=plan;
    conf.printPlan(&plan);    
    int n_v_2=conf.transitionSystem.m_vertices.size();
    if (n_v_2>n_v&& atoi(argv[4])<18){
        printf("difference=%i\n", n_v_2-n_v);
        return 1;
    }
    if (finished){
        printf("plan works");
        return 0;
    }
    else{
        return 1;
    }
    return !finished;
}