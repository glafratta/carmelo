#include "../callbacks.h"


int main(int argc, char** argv){
    //generate points
    int n_dimensions=1;
    if (argc>1){
        n_dimensions=atoi(argv[1]);
    }

    float pi_div_by=2;
    if (argc>2){
        pi_div_by=atof(argv[2]);
    }
    float angle=M_PI/pi_div_by;
    float min_d=0.01;
    float y_incr=min_d*sin(angle);
    float x_incr=min_d*cos(angle);

    std::vector <cv::Point2f> pts, box_points;
    float x=0, y=-0.05, width=0, length=0;
    for (int j=1; j<=n_dimensions; j++){
        for (int i=0; i<11; i++){
            pts.push_back(Pointf(x, y));
            y+=y_incr;
            x+=x_incr;
        }
        x+=min_d;
        y=-0.05;     
    }

    WorldBuilder wb;
    std::pair<bool,BodyFeatures> feature_bb= wb.bounding_box(pts);
    std::pair<bool,BodyFeatures> feature_rot= wb.wb_bridger.bounding_rotated_box(pts);
    feature_rot.second.halfLength=round(feature_rot.second.halfLength*10000)/10000;
    feature_rot.second.halfWidth=round(feature_rot.second.halfWidth*10000)/10000;
    if (fabs(feature_rot.second.halfLength*2-min_d*n_dimensions*10)>0.001){
        throw std::logic_error("wrong half length\n");
    }
    if (fabs(feature_rot.second.halfWidth-min_d*(n_dimensions-1))>0.001){
        throw std::logic_error("wrong half width\n");
    }
    if (!feature_rot.first){
        throw std::logic_error("feature does not exist\n");
    }
    return 0;
}