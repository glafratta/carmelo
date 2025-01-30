#include "../callbacks.h"


const std::map<StateMatcher::MATCH_TYPE, char*> match_map={{StateMatcher::_FALSE, "FALSE"}, {StateMatcher::_TRUE, "TRUE"}, {StateMatcher::ABSTRACT, "DISTURBANCE"}, {StateMatcher::POSE, "POSE"}, {StateMatcher::ANY, "ANY"}};

int main(){
StateMatcher matcher;
std::vector <std::vector <bool>>solution, result;
solution={{1, 0, 0, 0, 0}, {0, 1, 1, 1, 1}, {0, 0, 1, 0, 1}, {0,0,0,1,1}, {0,0,0,0,1}};
std::vector <StateMatcher::MATCH_TYPE> matches={StateMatcher::_FALSE, StateMatcher::_TRUE, StateMatcher::ABSTRACT, StateMatcher::POSE, StateMatcher::ANY};
   printf("o /d\t");
    for (int desired=0; desired<matches.size(); desired++){
        auto a=match_map.find(matches[desired]);
		printf("%s\t",(*a).second);
  
    }
    printf("\n");
    for (int observed=0; observed<matches.size(); observed++){
		auto a=match_map.find(matches[observed]);
		printf("%s\t",(*a).second);
        if ((*a).first!=StateMatcher::ABSTRACT){
            printf("\t");
        }
        std::vector <bool> row;
        for (int desired=0; desired<matches.size(); desired++){
            bool r=matcher.match_equal(matches[observed], matches[desired]);
            printf("%i\t", r) ;
            row.push_back(r);
        }
        printf("\n");
        result.push_back(row);
    }

    if (result!=solution){
        return 1;
    }
    return 0;
}