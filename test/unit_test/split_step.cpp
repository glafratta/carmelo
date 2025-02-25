#include "../callbacks.h"


int main(int argc, char** argv){
    Task goal=Task();
    Configurator conf(goal);
    conf.simulationStep=std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH)*2;
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    conf.data2fp.emplace(Pointf(atof(argv[1]), atof(argv[2])));
    conf.dummy_vertex(conf.movingVertex);
    auto v1 = boost::add_vertex(conf.transitionSystem);
    auto e1 = boost::add_edge(conf.currentVertex, v1, conf.transitionSystem);
    b2World world(b2Vec2(0.0, 0.0));
    conf.worldBuilder.buildWorld(world, conf.data2fp, b2Transform(b2Vec2(0,0), b2Rot(0)),DEFAULT);
    Task task(Disturbance(), DEFAULT);
    Robot robot(&world);
    simResult result=task.bumping_that(world, 1, robot.body);
    gt::fill(result, &conf.transitionSystem[v1], &conf.transitionSystem[e1.first]);
    int total_step=result.step, counted_step=0;
    std::vector <vertexDescriptor> split =conf.splitTask(v1, conf.transitionSystem, conf.transitionSystem[e1.first].direction, conf.currentVertex);
    for (vertexDescriptor v:split){ 
        auto ie = gt::inEdges(conf.transitionSystem,v , DEFAULT);
        if (!ie.empty()){
            counted_step+=conf.transitionSystem[ie[0]].step;
        }
    }
    printf("total=%i, counted%i\n", total_step, counted_step);

    return counted_step!=total_step;
}