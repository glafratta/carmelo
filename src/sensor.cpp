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




// b2Transform PointCloudProc::affineTransEstimate(std::vector <Pointf> current, Task::Action a,float timeElapsed, float range){
//         b2Transform result;
//         std::vector <Pointf> previousTmp = previous;
// 		if (previousTmp.empty() || current.empty() || previousTmp==current){
// 			return result;
// 		}
// 	 	float theta = a.getOmega()* timeElapsed;
// 		result.p ={a.getLinearSpeed()*cos(theta),a.getLinearSpeed()*sin(theta)};
// 		result.q.Set(a.getOmega());
// 		int diff = current.size()-previousTmp.size(); //if +ve,current is bigger, if -ve, previous is bigger

// 		if(diff>0){
//             for (int i=0; i<abs(diff); i++){
// 					previousTmp.push_back(previousTmp[0]); //before it was [-1]
// 			}

// 		}
// 		else if (diff<0){
//          for (int i=0; i<abs(diff); i++){
//                 current.push_back(current[0]);
//             }
// 		}
// 	//use partial affine transformation to estimate displacement
// 	cv::Mat transformMatrix =cv::estimateAffinePartial2D(previousTmp, current, cv::noArray(), cv::LMEDS);
// 	if (!transformMatrix.empty()){
// 		result.p.x= -(transformMatrix.at<double>(0,2))/timeElapsed;
// 		result.p.y = -(transformMatrix.at<double>(1,2))/timeElapsed;
// 		result.q.Set(acos(transformMatrix.at<double>(0,0))/timeElapsed);
// 		float posAngle=0; 
//         float tan = atan(result.p.y/result.p.x);//atan2 gives results between pi and -pi, atan gives pi/2 to -pi/2
// 		if (result.p.y !=0 && result.p.x !=0 && tan < MAX_OMEGA*timeElapsed){
// 			posAngle =tan;
// 		}
// 		if (result.p.Length()>MAX_SPEED){
// 			result.p.x = a.getLinearSpeed() *cos(posAngle);
// 			result.p.y = a.getLinearSpeed() *sin(posAngle);
// 		}

// 	}
// 	return result;

// }

// std::vector <Pointf> PointCloudProc::neighbours(b2Vec2 pos, float radius, std::vector <Pointf> data){ //more accurate orientation
// 	std::vector <Pointf> result= std::vector<Pointf>();
// 	float x1=pos.x-radius, x2=pos.x+radius, y1=pos.y-radius, y2=pos.y+radius;
// 	Pointf br(std::max(x1, x2), std::min(y1, y2));
// 	Pointf tl(std::min(x1, x2), std::max(y1, y2));
// 	if (data.empty()){
// 		data = previous;
// 	}
// 	for (Pointf p: data){
// 		if (p.isin(tl, br) & p!=getPointf(pos)){
// 			result.push_back(p);
// 		}
// 	}

// 	return result;
// }

// std::pair <bool, b2Vec2> PointCloudProc::findOrientation(std::vector<Pointf> vec){
// 	std::pair <bool, b2Vec2>result(false, b2Vec2());
// 	if (vec.size()<6){
// 		return result;
// 	}
// 	int count=0;
// 	float sumY=0, sumX=0;
// 	b2Vec2 avgVec;
// 	CompareY compareY;
// 	std::vector <Pointf> vec_copy(vec);
// 	Pointf p;
// 	std::vector<Pointf>::iterator pIt=vec.end();
// 	while(!vec.empty()){
// 		result.first=true;
// 		pIt =std::min_element(vec.begin(), vec.end(), compareY);
// 		p=*(pIt);
// 		vec_copy.push_back(p);
// 		vec.erase(pIt);
// 		auto pItNext=std::min_element(vec.begin(), vec.end(), compareY);
// 		Pointf p_next=*pItNext;
// 		float deltaY =p_next.y- p.y;
// 		float deltaX = p_next.x - p.x;
// 		count+=1;
// 		sumY+=deltaY;
// 		sumX+=deltaX;
// 	}
// 	avgVec.y = sumY/count;
// 	avgVec.x = sumX/count;
// 	avgVec.Normalize();
// 	result.second=avgVec;
// 	return result;
// }


// std::pair <bool, cv::Vec4f> PointCloudProc::findOrientationCV(std::vector<Pointf> vec){
// 	std::pair <bool, cv::Vec4f>result(false, 0);
// 	if (vec.size()<6){
// 		return result;
// 	}
// 	result.first=true;
// 	cv::Vec4f line; //vx, vy, x0, y0 -> (vx, vy) normalised collinear vector 
// 							    // -> (x0, y0) a point on the line
// 	cv::fitLine(vec, line, cv::DIST_L2, 0, 0.1, 0.1);
// 	result.second=line;
// 	return result;
// }

// std::vector<Pointf> PointCloudProc::setDisturbanceOrientation(Disturbance& d, CoordinateContainer data){
// 	//NEEDS TO BE SET at higher level than simulation (checking or exploring level, state matching level)
// 	std::vector <Pointf> v;
// 	if (!data.empty()){
// 		v=set2vec(data);
// 	}
// 	else{
// 		v=std::vector<Pointf>(previous);
// 	}
// 	std::vector <Pointf> nb=std::vector<Pointf>(neighbours(d.getPosition(), NEIGHBOURHOOD,v));
// 	std::pair<bool, b2Vec2> orientation =findOrientation(nb);	
// 	float dtheta=0;
// 	if (orientation.first){
// 		d.setOrientation(orientation.second.y, orientation.second.x);
// 	}
// 	return nb;

// }



// cv::Mat ImgProc::cropLeft(cv::Mat mat){
// 		float w=mat.size().width;
// 		float h=mat.size().height;
// 		cv::Mat result=mat(cv::Range(0, h), cv::Range(0, w/2));
//         return result;
// }

// cv::Mat ImgProc::cropRight(cv::Mat mat){
// 		float w=mat.size().width;
// 		float h=mat.size().height;
// 		cv::Mat result=mat(cv::Range(0, h), cv::Range(w/2, w));
//         return result;
// }

// cv::Vec2d ImgProc::opticFlow(const cv::Mat& frame){
// 		cv::Vec2d  optic_flow;
// 		cv::Mat frame_grey;
//         std::vector <cv::Point2f> new_corners;
//         std::vector <uchar> status;
//         std::vector<float> err;
//         cv::cvtColor(frame, frame_grey, cv::COLOR_RGB2GRAY);
//         if (corners.empty()){ //resample corners every 2 seconds (30fps)
//             cv::goodFeaturesToTrack(frame_grey, corners , gfp.MAX_CORNERS, gfp.QUALITY_LEVEL, gfp.MIN_DISTANCE);
//         }
//         if (it>0 & !corners.empty()){
//             cv::calcOpticalFlowPyrLK(previous, frame_grey, corners, new_corners, status, err); //no flags: error is L1 distance between points /tot pixels
//         }
//         else{
//             status=std::vector<uchar>(corners.size(), 1);
//         }

//         std::vector <cv::Point2f> good_corners;
//         int i=0;
//         for (i; i<corners.size();i++){
//             if (status[i]==1){
//                 good_corners.push_back(corners[i]); //og corners
//             }
//         }
// 		corners=good_corners;
// 		previous=frame_grey.clone();
//         if (!corners.empty()&!new_corners.empty()){ //corners are ordered from strongest to weakest
// 			optic_flow[0]=double(corners[0].x-new_corners[0].x);
//         	optic_flow[1]= double(corners[0].y-new_corners[0].y);
//         }

//         printf("updated %i\n", it);
//         it++;
// 		return optic_flow;

// }

// cv::Vec2d  ImgProc::avgOpticFlow(const cv::Mat& frame){
// 		cv::Vec2d  optic_flow;
// 		cv::Mat frame_grey;
//         std::vector <cv::Point2f> new_corners;
//         std::vector <uchar> status;
//         std::vector<float> err;
//         cv::cvtColor(frame, frame_grey, cv::COLOR_RGB2GRAY);
//         if (corners.empty()){ //resample corners every 2 seconds (30fps)
// 			cv::goodFeaturesToTrack(frame_grey, corners , gfp.MAX_CORNERS, gfp.QUALITY_LEVEL, gfp.MIN_DISTANCE);
//         //    printf("GFT, corners size=%i\n", corners.size());
//         }
//         if (!corners.empty() & !previous.empty()){
//             cv::calcOpticalFlowPyrLK(previous, frame_grey, corners, new_corners, status, err); //no flags: error is L1 distance between points /tot pixels
//           //  printf("LK\n");
//         }
//         else{
//             status=std::vector<uchar>(corners.size(), 1);
// 			//printf("corners %i, status %i\n", corners.size(), status.size());
//         }

//         std::vector <cv::Point2f> good_corners;
//         //if (it==1){
//         int i=0;
//         //printf("pre-fill in status, new corners size =%i\n", new_corners.size());
// 		for (i; i<corners.size();i++){
//             if (status[i]==1){
//                 good_corners.push_back(corners[i]); //og corners
// 				if (new_corners.size()==corners.size()){
// 					optic_flow[0]=double(corners[i].x-new_corners[i].x);
// 					optic_flow[1]=double(corners[i].y-new_corners[i].y);					
// 				}

//             }
//         }
// 		optic_flow[0]/=double(good_corners.size());
// 		optic_flow[1]/=double(good_corners.size());
// 		corners=good_corners;
// 		previous=frame_grey.clone();
//         it++;
// 		return optic_flow;

// }

// std::vector <cv::Point2f> ImgProc::get_corners(){
// 	return corners;
// }

// cv::Mat ImgProc::get_previous(){
// 	return previous;
// }