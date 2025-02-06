#include "../callbacks.h"



int main(int argc, char** argv){
    Disturbance goal(PURSUE, b2Vec2(1.0, 0));
    Task goal_t(goal, UNDEFINED);
    Configurator conf(goal_t);
    b2World world(b2Vec2(0,0));
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0)), d_pose=b2Transform(b2Vec2(0.55, 0), b2Rot(0));
    BodyFeatures bf(d_pose);
    bf.halfLength=0.02;
    bf.halfWidth=0.05;
    bf.attention=1;
    Disturbance obstacle(bf), solution=goal; 
    obstacle.validate(); 
    vertexDescriptor src;
    Direction src_d=DEFAULT, curr_d=DEFAULT;
    if (argc>2){
        src_d=Direction(atoi(argv[2]));
    } 
    conf.dummy_vertex(conf.movingVertex);
    //ASSUMING SRC ALREADY SIMULATED
    conf.transitionSystem[conf.currentEdge].direction=src_d;
    conf.transitionSystem[conf.currentVertex].Di=obstacle;
    if (argc>1){
        int code =atoi(argv[1]);
        if (code==0){
            start.p.x=0.40;
            solution=obstacle;
            conf.transitionSystem[conf.currentVertex].Di=goal;
            conf.transitionSystem[conf.currentVertex].Dn=obstacle;
        }
        else if (code==1){
            start.p.x=0.40;
            start.q.Set(M_PI_2);
            solution=obstacle;
        }
        else if (code==2){
            start.p.x=0.40;
            start.p.y=0.31;
            solution=obstacle;
        }
        else if (code==3){
            start.p.x=0.82;
            start.p.y=0.31;
        }
        else if (code==4){
            start.p.x=0.80;
            start.q.Set(-M_PI_2);
        }
    }
    conf.data2fp.emplace(getPointf(d_pose.p));
    conf.transitionSystem[conf.currentVertex].endPose=start;

   // conf.worldBuilder.buildWorld(world, conf.data2fp, start, curr_d, obstacle,0.15, WorldBuilder::PARTITION);
    //Robot robot(&world);
    //robot.body->SetTransform(start.p, start.q.GetAngle());
    //Task task(obstacle, DEFAULT, start,true);
    //task.makeRobotSensor(robot.body);
    if (argc>3){
        curr_d=Direction(atoi(argv[3]));
    }
    conf.transitionSystem[conf.currentVertex].options.push_back(curr_d);
// std::pair <edgeDescriptor, bool> edge=conf.add_vertex_now(src, v, conf.transitionSystem, obstacle);
    //conf.transitionSystem[v].options.push_back(curr_d);

    Disturbance result = conf.getDisturbance(conf.transitionSystem, conf.currentVertex, world, curr_d, b2Transform_zero);
    //debug_draw(world, 43);
    if (result==solution){
        return 0;
    }
    return 1;
}