#ifndef SENSOR_H
#define SENSOR_H
#include "task.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>

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


// class PointCloudProc{
// 	friend ConfiguratorInterface;
// 	friend Configurator;
//     std::vector <Pointf> previous;
// 	const float NEIGHBOURHOOD=0.075;
//     public:
//     PointCloudProc(){};

//     b2Transform affineTransEstimate(std::vector <Pointf>, Task::Action, float timeElapsed=0.2, float range=1.0);

// 	std::vector<Pointf> neighbours(b2Vec2,float radius, std::vector <Pointf> data= std::vector <Pointf>()); //finds if there are bodies close to a point. Used for 

// 	std::pair <bool, b2Vec2>  findOrientation(std::vector<Pointf> ); //finds  average slope of line passign through two points in a radius of 2.5 cm. Assumes low clutter 

// 	std::pair <bool, cv::Vec4f> findOrientationCV(std::vector<Pointf>);
	
// 	std::vector<Pointf> setDisturbanceOrientation(Disturbance&, CoordinateContainer data=CoordinateContainer());

// 	std::vector <Pointf> find_disturbance(const Disturbance &, const CoordinateContainer &);

// 	void updatePrevious(CoordinateContainer c){
// 		previous=set2vec(c);
// 	}
// };

// class ImgProc{
// 	std::vector <cv::Point2f> previousCorners;
//     public:
	
//     ImgProc(){}

//     cv::Mat cropLeft(cv::Mat);

//     cv::Mat cropRight(cv::Mat);

//     cv::Vec2d  opticFlow(const cv::Mat&);

// 	cv::Vec2d avgOpticFlow(const cv::Mat&);

// 	std::vector <cv::Point2f> get_corners();

// 	cv::Mat get_previous();

// 	void setPrevious(cv::Mat m){
// 		previous=m;
// 	}

// 	void setCorners(std::vector <cv::Point2f> c){
// 		corners=c;
// 	}

// 	void reset(){
// 		previous= cv::Mat();
// 		corners.clear();
// 	}


//     private:
// 	int it=0;
// 	struct GoodFeaturesParameters{
// 		const int MAX_CORNERS=30;
//     	const float QUALITY_LEVEL=0.7;
//    		const int MIN_DISTANCE=7;
//     	const int BLOCK_SIZE=7;
// 	}gfp;

//     std::vector <cv::Point2f> corners; //must be single-precision float
//     cv::Mat previous;
// };


#endif