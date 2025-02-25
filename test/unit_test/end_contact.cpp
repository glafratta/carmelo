#include "../callbacks.h"



int main(int argc, char** argv){
    Configurator conf;
    b2World world(b2Vec2(0,0));
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0)), d_pose=b2Transform(b2Vec2(0, -.35), b2Rot(0));
    conf.data2fp.emplace(getPointf(d_pose.p));
    BodyFeatures bf(d_pose);
    bf.halfLength=0.02;
    bf.halfWidth=0.05;
    bf.attention=1;
    Disturbance obstacle(bf); 
    Disturbance goal(PURSUE, b2Vec2(0.5,0.5));
    obstacle.validate();
    int expected=100;
    Task task;
    bool goal_d=atoi(argv[1]), goal_conf=atoi(argv[2]);
    if (!goal_d){
        task=Task(obstacle, DEFAULT, start,true);
        conf.worldBuilder.buildWorld(world, conf.data2fp, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    }
    if (goal_conf){
        Task goal_t=Task(goal, UNDEFINED);
        conf.controlGoal=goal_t;
        if (!goal_d){
            start.q.Set(M_PI_2);
            task.disturbance.bf.pose.p.x=-d_pose.p.y;
            task.disturbance.bf.pose.p.y=d_pose.p.x;
            expected=19;
            conf.worldBuilder.buildWorld(world, conf.data2fp, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
        }
        else{
            task=Task(goal, DEFAULT, start,true);
            expected=abs((start.p.x-goal.bf.pose.p.x))*HZ/task.action.getLinearSpeed();
        }
    }
    Robot robot(&world);
    b2AABB aabb= conf.worldBuilder.makeRobotSensor(robot.body, &conf.controlGoal.disturbance);
    debug_draw(world, 8);
    simResult sr= task.bumping_that(world, 69, robot.body, true);

    //linear speed =0.1
    printf("step=%i, expected=%i\n", sr.step, expected);
    if (sr.step!=expected){
        debug::print_pose(sr.endPose);
        return 1;
    }

    return 0;
}