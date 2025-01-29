#include "../callbacks.h"

void assign_disturbance(State & s, const b2Transform& t=b2Transform(b2Vec2(0.02,0.02), b2Rot(0)), float hw=0.0, float hl=0.0){
    BodyFeatures bf;
    bf.pose= s.endPose+t;
    bf.halfLength+=hl;
    bf.halfWidth+=hw;
    s.Dn=Disturbance(bf);
}

void rand_transform(b2Transform&t){
    srand(time(0));
    float r1= float(rand()%100+1)/100;
    float r2 = float(rand()%100+1)/100;
    float r3= float(rand()%314)/100;
    t+=b2Transform(b2Vec2(r1, r2), b2Rot(r3));
}

b2Transform rand_transform(){
    srand(time(0));
    float r1= float(rand()%100+1)/100;
    float r2 = float(rand()%100+1)/100;
    float r3= float(rand()%314)/100;
    b2Transform t=b2Transform(b2Vec2(r1, r2), b2Rot(r3));
    return t;
}

void create_match(State& candidate, StateMatcher::MATCH_TYPE mt, const State& s, int d_code=0){
    b2Transform transform= rand_transform();
    switch (mt)
    {
    case StateMatcher::_TRUE:
        candidate=State(s);
        break;
    case StateMatcher::D_NEW:
        switch (d_code)
        {
        case 0:
            candidate.start=s.start;
            candidate.endPose=s.endPose+transform;
            candidate.Dn=Disturbance(s.Dn); //exact match
            candidate.Dn.bf.pose=candidate.Dn.bf.pose+transform;
            break;
        case 1: //ratio +shape match
            rand_transform(candidate.endPose);
            assign_disturbance(candidate);
            candidate.Dn.bf.halfLength=s.Dn.bf.halfLength;
            candidate.Dn.bf.halfWidth=s.Dn.bf.halfWidth;
            break;
        case 2: //ratio match only //FAIL
            rand_transform(candidate.endPose);
            assign_disturbance(candidate, b2Transform(b2Vec2((0.02), (0.02)), ((b2Rot)(0))), 0.1, 0.1);
            break;
        case 3: //shape match only
            rand_transform(candidate.endPose);
            assign_disturbance(candidate, rand_transform());
            break;
        default:
            break;
        }
        break;
    case StateMatcher::POSE:
        candidate.endPose=b2Transform(s.endPose);
        break;
    default:
        break;
    }
}

void print_transform(const b2Transform& t){
    printf("x:%f, y=%f, theta=%f\n", t.p.x, t.p.y, t.q.GetAngle());
}

int main(int argc, char** argv){
    StateMatcher matcher;
    State s1, candidate;
    s1.start=b2Transform(b2Vec2(0.25, 0.25), b2Rot(M_PI_4));
    s1.endPose=b2Transform(b2Vec2(0.5, 0.5), b2Rot(M_PI_4));    
    s1.outcome=simResult::crashed;
    assign_disturbance(s1);
    s1.Dn.bf.halfWidth=0.05;
    s1.Dn.bf.halfLength=0.02;
    StateMatcher::MATCH_TYPE desired_match=StateMatcher::MATCH_TYPE::_FALSE;
    int d_code=0;
    printf("MATCH_TYPE {_FALSE=0, DISTURBANCE=2, POSE=3, _TRUE=1, ANY=4, D_POSE=5, D_SHAPE=6}\n");
    printf("D code = 0: exact, 1:  ratio+ shape, 2: ratio only, 3: shape\n");
    if (argc>2){
        printf("chosen D code= %i\n", atoi(argv[2]));
        d_code=atoi(argv[2]);
    }
    if (argc>1){
        desired_match=StateMatcher::MATCH_TYPE(atoi(argv[1]));
        printf("chosen desired match= %i\n", atoi(argv[1]));
        create_match(candidate, desired_match, s1, d_code);
    }
    printf("start: c ");
    print_transform(candidate.start);
    print_transform(s1.start);
    printf("end: ");
    print_transform(candidate.endPose);
    print_transform(s1.endPose);
    printf("disturbance: ");
    print_transform(candidate.Dn.pose());
    print_transform(s1.Dn.pose());
    StateMatcher::MATCH_TYPE result= matcher.isMatch(s1, candidate);
    if (result!=desired_match){
        printf(" desired =%i, result=%i\n", desired_match, result);
        return 1;
    }
    return 0;

}