#include "disturbance.h"


bool BodyFeatures::match(const BodyFeatures& bf){
    bool match_x=(pose.p.x-bf.pose.p.x)<D_POSE_MARGIN;
    bool match_y=(pose.p.y-bf.pose.p.y)<D_POSE_MARGIN;
    bool match_w=(halfWidth-bf.halfWidth)<D_DIMENSIONS_MARGIN;
    bool match_h=(halfLength-bf.halfLength)<D_DIMENSIONS_MARGIN;
    return match_x && match_y && match_w && match_h;
}

std::vector <b2Vec2> Disturbance::vertices()const{
    std::vector <b2Vec2> result;
    if (getAffIndex()==NONE){
        return result;
    }
    //assume orientation 0
    float pm_x=bf.halfWidth*cos(pose().q.GetAngle());
    float pm_y=bf.halfLength*sin(pose().q.GetAngle());
    result.push_back(b2Vec2(pose().p.x+pm_x,pose().p.y+pm_y));
    result.push_back(b2Vec2(pose().p.x-pm_x,pose().p.y+pm_y));
    result.push_back(b2Vec2(pose().p.x+pm_x,pose().p.y-pm_y));
    result.push_back(b2Vec2(pose().p.x-pm_x,pose().p.y-pm_y));
    return result;

}

float Disturbance::getAngle(b2Transform t){ //gets the angle of an Disturbance wrt to another Disturbance (robot)
        //reference is position vector 2. If the angle >0 means that Disturbance 1 is to the left of Disturbance 2
        float angle;
        b2Vec2 thisToB;
        thisToB.x = getPosition().x-t.p.x;
        thisToB.y = getPosition().y - t.p.y;
        float cosA = (thisToB.x * cos(t.q.GetAngle())+ thisToB.y*sin(t.q.GetAngle()))/thisToB.Length();
        angle = acos(cosA);
        return angle;
    }

void Disturbance::setOrientation(float s, float c){
    b2Rot og;
    og.s=s;
    og.c=c; 
    if (rotation_valid){
    b2Rot sup, comp, ver; //find most likely angle
        sup.s=-s;
        sup.c=c;
        comp.c=-c;
        comp.s=s;
        ver.s=-s;
        ver.c=-c;
        std::vector <b2Rot> rots={sup, comp, ver};
        for (b2Rot r:rots){
            if (fabs(r.GetAngle()-bf.pose.q.GetAngle())<fabs(og.GetAngle()-bf.pose.q.GetAngle())){
                og=r;
            }
        }
    }
    bf.pose.q.s=og.s;
    bf.pose.q.c=og.c;
    rotation_valid=1;
}


bool Disturbance::operator==(const Disturbance & d){
    bool _pose=bf.pose.p==d.bf.pose.p && bf.pose.q.GetAngle()==d.bf.pose.q.GetAngle();
    bool dim=halfLength()==d.bf.halfLength && halfWidth()==d.bf.halfWidth;
    bool aff=affordanceIndex==d.affordanceIndex;
}  

bool Disturbance::operator==(const Disturbance & d)const{
    bool _pose=bf.pose.p==d.bf.pose.p && bf.pose.q.GetAngle()==d.bf.pose.q.GetAngle();
    bool dim=bf.halfLength==d.bf.halfLength && bf.halfWidth==d.bf.halfWidth;
    bool aff=affordanceIndex==d.affordanceIndex;
}  

std::vector <b2Vec2> GetLocalPoints( std::vector <b2Vec2> pts, const b2Body * body){
	std::vector <b2Vec2> result;
	for (b2Vec2 p:pts){
		result.push_back(body->GetLocalPoint(p));
	}
	return result;
}