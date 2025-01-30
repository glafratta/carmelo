#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=0;
    Disturbance target1;
    if (argc>2){
        if (atoi(argv[2])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    std::vector <vertexDescriptor> options_src;
    State s1, s2;
    std::vector <BodyFeatures> bf1=conf.worldBuilder.getFeatures(ci.data2fp, s1.start, DEFAULT, BOX2DRANGE);
    s1.Dn= Disturbance(bf1[0]); //assumes 1 item length
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    math::applyAffineTrans(shift, s1);
    if (argc>3){
        di.iteration=atoi(argv[3]);
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    std::vector <BodyFeatures> bf2=conf.worldBuilder.getFeatures(ci.data2fp, s2.start, DEFAULT, BOX2DRANGE);
    s2.Dn= Disturbance(bf2[0]); //assumes 1 item length
    StateDifference sd(s2, s1);
    StateMatcher matcher;
    StateMatcher::StateMatch sm(sd, matcher.error);
    if (!sm.Dn_exact()){
        printf("sum_d=%f\n", sd.sum_D(sd.Dn));
        return 1;
    }
    return 0;
}