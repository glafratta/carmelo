#include "../callbacks.h"


int main(int argc, char** argv){
    //generate points
    std::vector <cv::Point2f> pts;
    float x=0, y=-0.05, width=0, length=0;
    for (int i=0; i<11; i++){
        pts.push_back(Pointf(x, y));
        y+=0.01;
    }
    WorldBuilder wb;
    std::pair<bool,BodyFeatures> feature_bb= wb.bounding_box(pts);
    std::pair<bool,BodyFeatures> feature_rot= wb.bounding_rotated_box(pts);
    feature_rot.second.halfLength=round(feature_rot.second.halfLength*1000)/1000;
    feature_rot.second.halfWidth=round(feature_rot.second.halfWidth*1000)/1000;

    if (fabs(feature_rot.second.halfLength-0.05)>0.001){
        throw std::logic_error("wrong half length\n");
    }
    if (feature_rot.second.halfWidth==0){
        throw std::logic_error("wrong half width\n");
    }
    if (feature_rot.second.pose.p.x !=0 || feature_rot.second.pose.p.y!=0){
        throw std::logic_error("wrong pose\n");
    }
    if (!feature_rot.first){
        throw std::logic_error("feature does not exist\n");
    }
    printf("w=:%f, h:%f", feature_rot.second.halfWidth, feature_rot.second.halfLength);
    return 0;
}