#ifndef TASK_H
#define TASK_H
#include "measurement.h"
const float SIM_DURATION = int(BOX2DRANGE*2 /MAX_SPEED);
const float WHEEL_SPEED_DEFAULT=0.5f;
const float WHEEL_SPEED_TURN=0.218182f;

b2Fixture * GetSensor(b2Body * body);

b2Body * GetDisturbance(b2World *);

bool overlaps(b2Body *, b2Body *);

bool overlaps(b2Body *, Disturbance *);


class Task{


public:
    friend class Configurator;
    char planFile[250]; //for debug
    bool debug_k=false; //delete this it's for debugging on the bhenchod pi
    b2Transform start;
    bool change =0;
    EndCriteria endCriteria; //end criteria other than task encounters a disturbance
    Direction direction= DEFAULT;
    int motorStep=0;
    int stepError=0;
    AffordanceIndex affordance=NONE;

struct Action{
private:
    float linearSpeed=WHEEL_SPEED_DEFAULT*2; //used to calculate instantaneous velocity using omega
    float recordedSpeed=linearSpeed;
    float omega=0; //initial angular velocity is 0
    float recordedOmega = omega;
    bool valid=0;
public:
    float R=WHEEL_SPEED_DEFAULT;
    float L=WHEEL_SPEED_DEFAULT;

    Action()=default;

    void init(Direction& direction){
        switch (direction){
        case Direction::DEFAULT:
        L=WHEEL_SPEED_DEFAULT;
        R=WHEEL_SPEED_DEFAULT;
        break;
        case Direction::LEFT:
        L=-WHEEL_SPEED_TURN;
        R=WHEEL_SPEED_TURN;
        break;
        case Direction::RIGHT:
        L=WHEEL_SPEED_TURN;//0.2537;
        R=-WHEEL_SPEED_TURN;
        break;
        default:
        L=0;
        R=0;
        break;
    }
    setVelocities(L, R);
    }

void setVelocities(float l, float r){
    omega = (MAX_SPEED*(r-l)/BETWEEN_WHEELS); //instant velocity, determines angle increment in willcollide
    recordedOmega = omega;
    linearSpeed = MAX_SPEED*(l+r)/2;
    recordedSpeed=linearSpeed;
    valid=1;
}

    b2Vec2 getLinearVelocity(){
        b2Vec2 velocity;
        velocity.x = linearSpeed *cos(omega);
        velocity.y = linearSpeed *sin(omega);
        return velocity;
    }

    b2Transform getTransform(){
    return b2Transform(getLinearVelocity(), b2Rot(omega));
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

    float getOmega(float l, float r){
        float result = (MAX_SPEED*(r-l)/BETWEEN_WHEELS)*TURN_FRICTION;
        return result;
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
    void setRec(float _speed, float _omega){
        recordedSpeed=_speed;
        recordedOmega=_omega;
    }

};



class Listener : public b2ContactListener {
 // int iteration=1;
    Disturbance * d_ptr;
    public:
    Listener(){}
    Listener(Disturbance * _d): d_ptr(_d){}
    std::vector <b2Body*> collisions;
        void BeginContact(b2Contact * contact) {
        b2Fixture * fixtureA= contact->GetFixtureA();
        b2Fixture * fixtureB= contact->GetFixtureB();
        b2BodyUserData bodyData = fixtureA->GetBody()->GetUserData();
        if (bodyData.pointer==ROBOT_FLAG) { //if fixtureA belongs to robot
            b2Body * other = contact->GetFixtureB()->GetBody();
            if (fixtureA->IsSensor()){
                if (other->GetUserData().pointer==DISTURBANCE_FLAG){
                    d_ptr->validate();
                }
                return;
            }
            if (fixtureB->IsSensor()){
                return;
            }
            collisions.push_back(other);
        }
        bodyData = fixtureB->GetBody()->GetUserData();
        if (bodyData.pointer==ROBOT_FLAG) {//WAS IF BODYDATA.POINTER if fixtureB belongs to robot
            b2Body * other = contact->GetFixtureA()->GetBody();
            if (fixtureB->IsSensor()){
                if (other->GetUserData().pointer==DISTURBANCE_FLAG){
                    d_ptr->validate();
                }
                return;
            }
            if (fixtureA->IsSensor()){
                return;
            }
            collisions.push_back(other);
            }       
        }

    //     void EndContact(b2Contact * contact) {
    //     b2Fixture * fixtureA= contact->GetFixtureA();
    //     b2BodyUserData bodyData = fixtureA->GetBody()->GetUserData();
    //     if (bodyData.pointer==ROBOT_FLAG) { //if fixtureA belongs to robot
    //         b2Body * other = contact->GetFixtureB()->GetBody();
    //         if (fixtureA->IsSensor() &&other->GetUserData().pointer==DISTURBANCE_FLAG){
    //             d_ptr->invalidate();
    //         }
    //     }
    //     b2Fixture * fixtureB= contact->GetFixtureB();
    //     bodyData = fixtureB->GetBody()->GetUserData();
    //     if (bodyData.pointer==ROBOT_FLAG) {//WAS IF BODYDATA.POINTER if fixtureB belongs to robot
    //         b2Body * other = contact->GetFixtureA()->GetBody();
    //         if (fixtureB->IsSensor() &&other->GetUserData().pointer==DISTURBANCE_FLAG){
    //             d_ptr->invalidate();
    //         }
    //     }       
    // }
        
	};
	
    class Query : public b2QueryCallback {
		Disturbance * d_ptr=NULL;
        public:
            Query(Disturbance * _d_ptr):d_ptr(_d_ptr){}
            std::vector<b2Body*> d;
            
            bool ReportFixture(b2Fixture* fixture) {
                if (fixture->GetBody()->GetUserData().pointer==DISTURBANCE_FLAG){
                    //d.push_back( fixture->GetBody() ); 
                    if (d_ptr==NULL){
                        return false;
                    }
                    d_ptr->invalidate();
                    return true;//keep going to find all fixtures in the query area
                }
                return false;
            }
    };



public:
Action action;

Disturbance disturbance;

Task::Action getAction(){
    return action;
}

AffordanceIndex getAffIndex(){
    return affordance;
}


Direction H(Disturbance, Direction, bool topDown=0); //topDown enables Configurator topdown control on reactive behaviour


void setEndCriteria(Angle angle=SAFE_ANGLE, Distance distance=BOX2DRANGE);

void setErrorWeights();

EndedResult checkEnded(b2Transform robotTransform = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), Direction dir=UNDEFINED, bool relax=0, b2Body* robot=NULL, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0))));

EndedResult checkEnded(State, Direction dir=UNDEFINED, bool relax=false, std::pair<bool,b2Transform> use_start= std::pair <bool,b2Transform>(1, b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)))); //usually used to check against control goal

Task(){
    start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    direction = DEFAULT;
    action.init(direction);
}

Task(Direction d){
    direction=d;
    action.init(direction);
}

Task(Disturbance ob, Direction d, b2Transform _start=b2Transform(b2Vec2(0.0, 0.0), b2Rot(0.0)), bool topDown=0){
    start = _start;
    disturbance = ob;
    affordance=disturbance.getAffIndex();
    direction = H(disturbance, d, topDown);  
    action.init(direction);
    setEndCriteria();
}


simResult willCollide(b2World &, int, b2Body *,bool debug =0, float remaining = SIM_DURATION, float simulationStep=BOX2DRANGE);

EndCriteria getEndCriteria(const Disturbance&);
};

#endif