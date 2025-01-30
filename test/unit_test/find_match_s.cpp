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
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    math::applyAffineTrans(shift, conf.transitionSystem);    
    if (argc>4){
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    std::vector <BodyFeatures> b_features=conf.worldBuilder.getFeatures(conf.data2fp, state_tmp.start, DEFAULT, BOX2DRANGE);
    if (!b_features.empty()){
        state_tmp.Dn= Disturbance(b_features[0]); //assumes 1 item length
    }
    bool relax_match=1;
    conf.addIteration();
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.findMatch(state_tmp,conf.transitionSystem, NULL, UNDEFINED, StateMatcher::D_NEW, &options_src, relax_match);
    if (options_src.empty()){
        return 1;
    }
    for (vertexDescriptor o: options_src){
        if (o==solution){
            return 0;
        }

    }
    for (vertexDescriptor o: options_src){
        //if (o==solution){
            printf("match with =%i\n",  o );
        //}

    }
    return 2;
}