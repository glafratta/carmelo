#include "../callbacks.h"

bool debug_draw(b2World & w, int file, char * label){
    char name_b[256], name_v[256];
    sprintf(name_b, "/tmp/debug_bodies_%s_%i.txt", label, file);
    sprintf(name_v, "/tmp/debug_vertices_%s_%i.txt",label, file);
    FILE * f_b=fopen(name_b, "w");
    FILE * f_v=fopen(name_v, "w");
    for (auto b=w.GetBodyList(); b; b=b->GetNext()){
        //if (!b->GetUserData().pointer){
            fprintf(f_b, "%f\t%f\n", b->GetPosition().x, b->GetPosition().y);
            b2PolygonShape * shape= (b2PolygonShape*)b->GetFixtureList()->GetShape();
            int ct= shape->m_count;
            for (int i=0; i<shape->m_count; i++){
                b2Vec2* v=shape->m_vertices+i;
                b2Vec2 world_point= b->GetWorldPoint(*v);
                fprintf(f_v, "%f\t%f\n",  world_point.x, world_point.y);            
            }
        //}
    }
    fclose(f_b);
    fclose(f_v);
}


int main(int argc, char** argv){
    Configurator conf;
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
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    b2Transform start;
    Task task(DEFAULT);
    conf.worldBuilder.buildWorld(world, conf.data2fp, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    debug_draw(world, atof(argv[5]), "first");
    Robot robot(&world);
    simResult sim= task.bumping_that(world, conf.getIteration(), robot.body);
    Disturbance obstacle=sim.collision; 
    if (argc>4){
        start.p.x=atof(argv[2]);
        start.p.y=atof(argv[3]);
        start.q.Set(atof(argv[4]));
    }
    Task task2(obstacle, DEFAULT, start,true);
    conf.worldBuilder.buildWorld(world, conf.data2fp, task2.start, task2.direction, task2.disturbance, 0.15, WorldBuilder::PARTITION);
    debug_draw(world, atof(argv[5]), "avoid");
    return 0;
}