#include "worldbuilder.h"
#include <fstream>

std::vector <BodyFeatures> WorldBuilder::processData( const CoordinateContainer &pts, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <cv::Point2f> points, centers;
    for (Pointf p:pts){
        points.push_back(cv::Point2f(float(p.x), float(p.y)));
    }
    //std::vector <cv::Point2f> centers;
    std::vector<std::vector<cv::Point2f>> clusters=kmeans_clusters(points, centers);
    for (int i=0;i<clusters.size(); i++){
        char filename[50];
        sprintf(filename, "/tmp/cluster%04i.txt", i+1);
        FILE * f=fopen(filename, "w+");
        for (int j=0; j<clusters[i].size(); j++){
            fprintf(f, "%f\t%f\n", clusters[i][j].x, clusters[i][j].y);
        }
        fclose(f);
    }
    for (int c=0; c<clusters.size(); c++){
        if (std::pair<bool,BodyFeatures>feature=bounding_box(clusters[c]); feature.first){
            feature.second.pose.q.Set(start.q.GetAngle());
            result.push_back(feature.second);
        }
    }
    return result;

}

int main(int argc, char** argv){
    WorldBuilder wb;
    WorldBuilder::CompareCluster cc;
    CoordinateContainer points;
    char filePath[256];
    sprintf(filePath, "%smap%04i.dat", argv[1], atoi(argv[2]));
    printf("%s\n", filePath);
    std::ifstream file(filePath);
    float x2, y2;
    while (file>>x2>>y2){
        x2 = round(x2*100)/100;
        y2 = round(y2*100)/100;
        Pointf  p2(x2,y2);
        points.insert(p2);
    }
    file.close();
    float x=0, y=0, r=0;
    if( argc>5){
        x=atof(argv[3]);
        y=atof(argv[4]);
        r=atof(argv[5]);
    }
    b2Transform start=b2Transform(b2Vec2(x,y), b2Rot(r));
    std::vector <BodyFeatures> features =wb.processData(points, start);
    return 0;


}