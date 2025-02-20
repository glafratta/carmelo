#include "sensor.h"

bool Pointf::isin(Pointf tl, Pointf br){
	bool result= this->x>tl.x & this->x<br.x & this->y>br.y& this->y<tl.y;
	return result;
}


float length(cv::Point2f const& p){
	return sqrt(pow(p.x,2)+ pow(p.y, 2));
}


float angle(cv::Point2f const& p){
	return atan2(p.y, p.x);
}

bool operator <(Pointf const & p1, Pointf const& p2){
	float a1 = angle(p1);
	float l1=length(p1);
	float a2=angle(p2);
	float l2=length(p2); 
	return std::tie(a1, l1)< std::tie(a2, l2);
}	

bool operator >(const Pointf& p1,  const Pointf& p2){
	return p2<p1;
}

b2Vec2 getb2Vec2(cv::Point2f p){
	return b2Vec2(p.x,p.y);

}


Pointf Polar2f(float radius, float angle){
	float x = radius *cos(angle);
	float y = radius *sin(angle);
	return Pointf(x,y);
}
