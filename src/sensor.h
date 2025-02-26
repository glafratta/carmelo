#ifndef SENSOR_H
#define SENSOR_H
#include "task.h"
// #include <opencv2/imgproc.hpp>
// #include <opencv2/tracking.hpp>

class ConfiguratorInterface;
class Configurator;

class Pointf: public cv::Point2f{
	public: 

	Pointf(){}

	Pointf(float _x, float _y){
		x=_x;
		y=_y;
	}
    
    Pointf operator+(const Pointf p){
			Pointf result;
			result.x = x + p.x;
			result.y = y+ p.y;
			return result;
	}

	Pointf operator-(const Pointf p){
			Pointf result;
			result.x = x - p.x;
			result.y = y- p.y;
			return result;
	}

	bool isin(Pointf, Pointf);

};


template<>
struct cv::traits::Depth<Pointf> {enum {value = Depth<cv::Point2f>::value};};

template<>
struct cv::traits::Type<Pointf> {enum {value = CV_MAKETYPE(cv::traits::Depth<Pointf>::value, 2)};};

float length(cv::Point2f const& p);

float angle(cv::Point2f const&);

bool operator <(Pointf const &, Pointf const&);

bool operator >(const Pointf&,  const Pointf&);

typedef std::set<Pointf> CoordinateContainer;


b2Vec2 getb2Vec2(cv::Point2f );

template <typename T>
Pointf getPointf(T v){
	return Pointf(v.x, v.y);
}

Pointf Polar2f(float, float);

template <typename T>
std::vector<T> set2vec(std::set<T> s){
    std::vector <T> vec;
    for (T t:s){
        vec.emplace_back(t);
    }
    return vec;
}

template <typename T>
std::vector<cv::Point2f> set2vec2f(std::set<T> s){
    std::vector <cv::Point2f> vec;
    for (T t:s){
        vec.push_back(cv::Point2f(t.x, t.y));
    }
    return vec;
}

template <typename T>
std::set<T> vec2set(std::vector<T> vec){
	std::set <T> set;
    for (T t:vec){
        set.emplace(t);
    }
    return set;
}

#endif