#ifndef AFFORDANCE_H
#define AFFORDANCE_H
#include <vector>
#include <stdio.h>

typedef unsigned int AffordanceIndex; //was thinking of this being a character but doesn't have to be maybe enum is fine
enum InnateAffordances {NONE, AVOID, PURSUE}; //for ease of identification, previously also ATTACK and EXPLORE
enum Direction{LEFT, RIGHT, DEFAULT, BACK, STOP, UNDEFINED};



// class Affordance{
// public:
//     AffordanceIndex ID=-1;
//     std::vector <Direction> options; //vector of primitives, low level affordance
//     Affordance(){}

//     Affordance(std::vector <Direction> vec, AffordanceIndex i =-1):ID(i){
//         for (Direction d:vec){
//             options.push_back(d);
//         }
//     }

//     void initialiseMs(std::vector<Direction> vec){
//         for (Direction d:vec){
//             options.push_back(d);
//         }
//     }

//     void setID(AffordanceIndex i){
//         ID =i;
//     }
//     //can map up or down in a hierarchy (not implemented) through pointers or maps
    
// };


#endif