#ifndef TASK_H
#define TASK_H
//#include "box2d/box2d.h"
#include <vector>
#include <stdio.h>
#include <math.h> 
//#include "general.h"
#include "measurement.h"
const float SIM_DURATION = int(BOX2DRANGE*2 /MAX_SPEED);
#define SAFE_DISTANCE sqrt(ROBOT_HALFLENGTH*ROBOT_HALFLENGTH+ROBOT_HALFWIDTH*ROBOT_HALFWIDTH) //DEFAULT backtracking distance

//struct Node;

class Task{
public:
    friend class Configurator;
    float accumulatedError=0;
    char planFile[250]; //for debug
    b2Transform start;
    //b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    bool change =0;
    float pGain=0.063;
    //float endAvoid = M_PI_2;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction;
protected:
    //Type type = Type::BASELINE;
    b2Vec2 RecordedVelocity ={0.0f, 0.0f};
public:
struct Action{
private:
    float linearSpeed=.0625; //used to calculate instantaneous velocity using omega
    float recordedSpeed=linearSpeed;
    float omega=0; //initial angular velocity is 0
    float recordedOmega = omega;
    bool valid=0;
public:
    float R=.5;
    float L=.5;

    Action()=default;

    void init(Direction direction){
        switch (direction){
        case Direction::DEFAULT:
        L=0.5;
        R=.5;
        break;
        case Direction::LEFT:
        L = -0.5;
        R=0.5;
        break;
        case Direction::RIGHT:
        L=0.5;
        R = - 0.5;
        break;
        case Direction::BACK:
        L = -0.5;
        R = -0.5;
        break;
        case Direction::STOP:
        L=0;
        R=0;
        break;
        default:
        throw std::invalid_argument("not a valid direction for M");
        break;
    }
    //kinematic model internal to action so it can be versatile for use in real P and simulated P
    omega = (MAX_SPEED*(R-L)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
    recordedOmega = omega;
    linearSpeed = MAX_SPEED*(L+R)/2;
    recordedSpeed=linearSpeed;

    valid=1;
        
    }


    b2Vec2 getLinearVelocity(){
        b2Vec2 velocity;
        velocity.x = linearSpeed *cos(omega);
        velocity.y = linearSpeed *sin(omega);
        return velocity;
    }

    float getRWheelSpeed(){
        return R;
    }

    float getLWheelSpeed(){
    return L;
    }


    bool isValid(){
        return valid;
    }

    float getLinearSpeed(){
        return linearSpeed;
    }

    float getOmega(){
    return omega;
    }

    void setOmega(float o){
        omega =o;
    }

    void setLinearSpeed(float s){
        linearSpeed =s;
    }

    void setRecSpeed(float s){
        recordedSpeed =s;
    }

    void setRecOmega(float w){
        recordedOmega=w;
    }

    float getRecSpeed(){
        return recordedSpeed;
    }

    float getRecOmega(){
        return recordedOmega;
    }
    //friend class Configurator;
};



class Listener : public b2ContactListener {
  int iteration=1;
    public:
    Listener(){}
    std::vector <b2Vec2> collisions;
    
		void BeginContact(b2Contact * contact) {
			b2BodyUserData bodyData = contact->GetFixtureA()->GetBody()->GetUserData();
			if (bodyData.pointer) {
                b2Body * other = contact->GetFixtureB()->GetBody();
                collisions.push_back(other->GetPosition());
			}
			bodyData = contact->GetFixtureB()->GetBody()->GetUserData();
			if (bodyData.pointer) {
                b2Body * other = contact->GetFixtureA()->GetBody();
                collisions.push_back(other->GetPosition());
                }       
		}
        
	};
private:
Action action;
public:
//std::vector <Direction> options;
Disturbance disturbance;

Task::Action getAction(){
    return action;
}

AffordanceIndex getAffIndex(){
    return disturbance.getAffIndex();
}

Direction H(Disturbance, Direction);

void setEndCriteria();

void setErrorWeights();

EndedResult checkEnded(b2Transform robotTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)));

EndedResult checkEnded(Node);
//Task(){}

Task(){
    start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    direction = DEFAULT;
    action.init(direction);
    //RecordedVelocity = action.getLinearVelocity();
}

Task(Disturbance ob, Direction d, b2Transform _start=b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0))){
    start = _start;
    disturbance = ob;
    direction = H(disturbance, d);  
    //action = Action(direction);
    action.init(direction);
    //RecordedVelocity = action.getLinearVelocity();
    setEndCriteria();
}

void init(){
    start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    direction = DEFAULT;
    action.init(direction);
   // RecordedVelocity = action.getLinearVelocity();
}

void init(Disturbance ob, Direction d, b2Transform _start=b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0))){
    start = _start;
    disturbance = ob;
    direction = H(disturbance, d);  
    //action = Action(direction);
    action.init(direction);
   // RecordedVelocity = action.getLinearVelocity();
    setEndCriteria();
}

void setRecordedVelocity(b2Vec2 vel){
    RecordedVelocity = vel;
    
} //useful to get the speed.


b2Vec2 getRecordedVelocity(){
    return RecordedVelocity;
}


void trackDisturbance(Disturbance &, float, b2Transform, b2Transform= b2Transform(b2Vec2(0,0), b2Rot(0)));

simResult willCollide(b2World &, int, bool, float);

enum controlResult{DONE =0, CONTINUE =1};

controlResult controller();

void setGain(float f){
    pGain=f;
}

std::pair<bool, b2Vec2> findNeighbourPoint(b2World &, b2Vec2, float radius = 0.02); //finds if there are bodies close to a point. Used for 
                                                                                    //finding a line passing through those points

float findOrientation(b2Vec2, b2Vec2); //finds slope of line passign through two points


};

#endif