#ifndef DISTURBANCE_H
#define DISTURBANCE_H
#include "settings.h"
#include "robot.h"
#include <algorithm>
#include <stdexcept>

struct CompareY{
	template <typename T>
    bool operator() ( T a, T b ){ //
        return a.y <=b.y;
	}
}; 

struct CompareX{
    template <typename T>
	bool operator()(T a, T b){
		return a.x<=b.x;
	}
};

template <typename C>
std::vector <C> arrayToVec(C* c, int ct){
	std::vector <C> result;
	for (int i=0; i<ct; i++){
		result.push_back(*c);
		c++;
	}
	return result;
}


class Configurator;

class BodyFeatures{
    public:
    b2Transform pose {b2Transform(b2Vec2(0,0), b2Rot(0))} ;
   // b2Transform pose_local=pose;
    float halfLength=MIN_BODY_DIMENSION;//x
    float halfWidth=MIN_BODY_DIMENSION; //y
    float shift=0.0f;
    b2BodyType bodyType = b2_dynamicBody;

    b2Shape::Type shape = b2Shape::e_polygon;
    //std::vector<b2Vec2> vertices;
    bool attention=false;

    BodyFeatures(){}

    BodyFeatures(b2Transform _pose):pose(_pose){}

    void setHalfLength(const float & f){
        if (f<MIN_BODY_DIMENSION){
            halfLength=MIN_BODY_DIMENSION;
        }
        else{
            halfLength=f;
        }
    }

    void setHalfWidth(const float & f){
        if (f<MIN_BODY_DIMENSION){
            halfWidth=MIN_BODY_DIMENSION;
        }
        else{
            halfWidth=f;
        }
    }

    bool match(const BodyFeatures&);

    float width()const{
        return halfWidth*2;
    }

    float length()const{
        return halfLength*2;
    }

};

struct Disturbance{ //this generates error

private:
friend Configurator;
friend struct StateMatcher;
    AffordanceIndex affordanceIndex = NONE; //not using the enum because in the future we might want to add more affordances
    bool valid= 0;
    bool rotation_valid=0;    

    void setOrientation(float f){ //returns orientation (angle) of a point, in order 
        rotation_valid=1;
        bf.pose.q.Set(f);
    }

    void addToOrientation(float dtheta){
    if (rotation_valid){
        setOrientation(bf.pose.q.GetAngle()+dtheta);
    }
    else{
        setOrientation(dtheta);
    }
}

public:
    BodyFeatures bf=BodyFeatures(b2Transform(b2Vec2(2*BOX2DRANGE, 2*BOX2DRANGE), b2Rot(M_PI)));

    Disturbance(){};
    Disturbance(AffordanceIndex i){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
    }
    Disturbance(AffordanceIndex i, b2Vec2 p){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
		bf.pose.Set(p, 0);
        valid =1;
    }    

        Disturbance(AffordanceIndex i, b2Vec2 p, float a){
        if (i>affordances.size()-1){
            throw std::invalid_argument("Not a valid affordance index\n");
        }
        else{
            affordanceIndex = i;
        }
        bf.pose.Set(p,a);
        valid =1;
    }   

    Disturbance(BodyFeatures _bf): bf(_bf){
       // valid=1;
        affordanceIndex=1;
    } 

    Disturbance(b2Body* b){
        bf.pose=b->GetTransform(); //global
        b2Fixture * fixture =b->GetFixtureList();
        bf.shape=(fixture->GetShape()->GetType());
        valid=1;
        if (bf.shape==b2Shape::e_polygon){
            b2PolygonShape * poly=(b2PolygonShape*)fixture->GetShape();
            std::vector <b2Vec2> local_vertices=arrayToVec(poly->m_vertices, poly->m_count);
            CompareX compareX;
            CompareY compareY;
            float minx=(std::min_element(local_vertices.begin(), local_vertices.end(), compareX)).base()->x;
            float miny=(std::min_element(local_vertices.begin(), local_vertices.end(), compareY)).base()->y;
            float maxx=(std::max_element(local_vertices.begin(), local_vertices.end(), compareX)).base()->x;
            float maxy=(std::max_element(local_vertices.begin(), local_vertices.end(), compareY)).base()->y;
            bf.halfLength=(fabs(maxy-miny))/2; //local coordinates
            bf.halfWidth=(fabs(maxx-minx))/2;
        }
        bf.attention=true;
        affordanceIndex=1;
    }

    float getAngle(b2Transform);

    float getAngle(b2Body* b){
        return getAngle(b->GetTransform());
    }

    void setPosition(b2Vec2 pos){
        bf.pose.p.Set(pos.x, pos.y);
    }
    
    void setPosition(float x, float y){
        bf.pose.p.Set(x, y);
    }
    
    b2Vec2 getPosition(){
        return bf.pose.p;
    }


    bool isValid()const{
        return valid;
    }

    AffordanceIndex getAffIndex()const{
        return affordanceIndex;
    }

    void invalidate(){
        valid =0;
    }

    void validate(){
        valid=1;
    }


    std::pair<bool, float> getOrientation(){
    
        return std::pair<bool, float>(rotation_valid, bf.pose.q.GetAngle());
    }

    b2Transform pose()const{
        return bf.pose;
    }

    void setPose(b2Transform t){
        bf.pose=t;
    }

    void setAsBox(float w, float l){
        bf.halfLength=l;
        bf.halfWidth=w;
    }

    BodyFeatures bodyFeatures()const{
        return bf;
    }


    void subtractPose(b2Transform dPose){
        bf.pose.p.x-=dPose.p.x;
        bf.pose.p.y-=dPose.p.y;
        addToOrientation(-dPose.q.GetAngle());
    }

    float halfLength(){
        return bf.halfLength;
    }

    float halfWidth(){
        return bf.halfWidth;
    }

    void setOrientation(float, float);

    std::vector <b2Vec2> vertices(); //global vertices

    bool operator==(const Disturbance & d);

    bool operator==(const Disturbance & d)const;

}; //sub action f


struct simResult{
    enum resultType {successful =0, crashed =1, safeForNow=2}; //successful means no collisions, finished means target reached, for later
    resultType resultCode= resultType::successful;
    Disturbance collision;
    //bool valid = 0;
    b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0));
    int step=0;


    simResult(){}

    simResult(resultType code): resultCode(code){
     //   valid =1;
    }

    simResult(resultType code, Disturbance obst): resultCode(code), collision(obst){
       // valid =1;
    }
};

std::vector <b2Vec2> GetLocalPoints( std::vector <b2Vec2>, const b2Body *);

#endif