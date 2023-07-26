#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include "robot.h"

class Measurement{
protected:
    bool valid =0;
    float value=0;
public:
    Measurement(){}

    bool isValid(){
        return valid;
    }

    float get(){
        return value;
    }

    void set(float f){
        value =f;
    }

    bool operator<(Measurement &);

    bool operator>=(Measurement &);

    float getError(Measurement);

    float getStandardError(Measurement); //relative standard error

};

class Angle: public Measurement{
    public:
    Angle(){}
    Angle(float f)
    {   value =round(f*100)/100;
        valid =1;}
};

class Distance: public Measurement{
    public:
    Distance(){}
    Distance(float f)
    {value = round(f*100)/100;
    valid =1;}
};


struct EndCriteria{
    Angle angle;
    Distance distance;    
    float getError(EndCriteria); //expresses magnitude of error, not normalised
    float getStandardError(EndCriteria);
    float getStandardError(Angle, Distance);
   // float getStandardError(Node);
    bool hasEnd();

};


struct EndedResult{
	bool ended=0;
	float errorFloat=0; //dot product of end criteria

    EndedResult() = default;
};

//typedef Measurement Mst;
// bool operator<(Meas &, Meas &);

// bool operator>=(Meas &, Meas &);


float SignedVectorLength(b2Vec2);

#endif