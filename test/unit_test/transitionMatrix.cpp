#include "../callbacks.h"
#include <string>

Direction getDirection(char d){
    
    if (d=='l'){
        return LEFT;
    }
    else if (d=='r'){
        return RIGHT;
    }
    else if (d=='d'){
        return DEFAULT;
    }
    else if (d=='s'){
        return STOP;
    }
    else if (d=='u'){
        return UNDEFINED;
    }
    else{
        throw std::invalid_argument("invalid direction");
    }
}

simResult::resultType getOutcome(char o){
    if (o=='c'){
        return simResult::crashed;
    }
    else if (o=='s'){
        return simResult::safeForNow;
    }
    else if (o=='k'){
        return simResult::successful;
    }
    else{
        throw std::invalid_argument("invalid outcome");
    }
}

int expectedOptions(Direction dir, simResult::resultType o, Disturbance d){
    if (o==simResult::safeForNow){
        if(dir==DEFAULT || dir==STOP){
            return 2;
        }
    }
    else if (o==simResult::successful){
        if (dir==LEFT || dir==RIGHT){
            if (d.getPosition().x<0){
                return 2;
            }
            else{
                return 1;
            }
        }
        else{
            if (d.isValid() && d.getPosition().y !=0){
                return 3;
            }
            else{
                return 1;
            }
        }

    }
    else if (o==simResult::crashed){
        return 0;
    }
}

void print_directions(std::vector <Direction> vec){
    for (Direction d:vec){
        auto a=dirmap.find(d);
		printf("%s, ",(*a).second);
    }
}


int main(int argc, char** argv){
    
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
    bool debug=0;
    Disturbance target1;
    if (argc>1){
        if (atoi(argv[1])==1 ){
            if (argc>6){
                float x=atof(argv[4]);
                float y=atof(argv[5]);
                float theta=atof(argv[6]);
                target1= Disturbance(PURSUE, b2Vec2(x,y), theta);
            }
            else{
                target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
            }
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    vertexDescriptor v1=boost::add_vertex(conf.transitionSystem);
    auto e=boost::add_edge(conf.currentVertex, v1, conf.transitionSystem);
    conf.transitionSystem[e.first].step=1;
    Direction direction=DEFAULT;
    simResult::resultType outcome=simResult::successful;
    if (argc>2){
        direction=getDirection(*argv[2]);
    }
    if (argc>3){
        outcome=getOutcome(*argv[3]);
    }
    conf.transitionSystem[e.first].direction=direction;
    conf.transitionSystem[v1].outcome=outcome;
    int expected_options=expectedOptions(direction, outcome, target1);
    conf.applyTransitionMatrix(conf.transitionSystem, v1, direction, false, conf.currentVertex,conf.planVertices );
    printf("expected options=%i\n", expected_options);
    print_directions(conf.transitionSystem[v1].options);
    if (int(conf.transitionSystem[v1].options.size())!=expected_options){
        throw std::logic_error("wrong options");
    }
    return 0;
}