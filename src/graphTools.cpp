#include "graphTools.h"

orientation subtract(orientation o1, orientation o2){
	orientation result;
	if (!o1.first){
		o1.second=0;
	}
	if (!o2.first){
		o2.second=0;
	}
	result.first= o1.first ||o2.first;
	result.second=o1.second-o2.second;
	return result;
}

b2Transform State::start_from_Di()const{
	if (Di.getAffIndex()==NONE){
		return b2Transform_inf;
	}
	return Di.pose()-start; //START
}

b2Transform State::end_from_Dn()const{
	if (Dn.getAffIndex()==NONE){
		return b2Transform_inf;
	}
	return Dn.pose()-endPose; //START
}

float State::distance(){
	return (start-endPose).p.Length();
}


float angle_subtract(float a1, float a2){
	float result = 0;
	if (fabs(a1)> 3*M_PI_4 || fabs(a2)> 3*M_PI_4){
		if (a1<0 & a2>0){
			a2-=2*M_PI;
		}
		else if(a2<0 & a1>0){
			a2+=2*M_PI;
		}
	}
	result=a1-a2;
	return result;
}

void math::applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose){
	pose.q.Set(pose.q.GetAngle()+deltaPose.q.GetAngle());
	float og_x= pose.p.x, og_y=pose.p.y;
	pose.p.x= og_x* cos(deltaPose.q.GetAngle())+ og_y*sin(deltaPose.q.GetAngle());
	pose.p.y= og_y* cos(deltaPose.q.GetAngle())- og_x*sin(deltaPose.q.GetAngle());
	// pose.p.x= og_x* cos(pose.q.GetAngle())+ og_y*sin(pose.q.GetAngle());
	// pose.p.y= og_y* cos(pose.q.GetAngle())- og_x*sin(pose.q.GetAngle());
	pose.p.x+=deltaPose.p.x; //-
	pose.p.y+=deltaPose.p.y; //-
}

void math::applyAffineTrans(const b2Transform& deltaPose, State& state){
	applyAffineTrans(deltaPose, state.endPose);
	applyAffineTrans(deltaPose, state.start);
	if (state.Dn.getAffIndex()!=NONE){
		applyAffineTrans(deltaPose, state.Dn.bf.pose);
	}
	if (state.Di.getAffIndex()!=NONE){
		applyAffineTrans(deltaPose, state.Di.bf.pose);
	}

}


void math::applyAffineTrans(const b2Transform& deltaPose, TransitionSystem& g){
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
	if (*vIt!=0){
		math::applyAffineTrans(deltaPose, g[*vIt]);
	}
}
}

void math::applyAffineTrans(const b2Transform& deltaPose, Disturbance& d){
	if (d.getAffIndex()!=NONE){
		math::applyAffineTrans(deltaPose, d.bf.pose);
	}
}



float StateDifference::get_sum(int mt){
	if (mt==StateMatcher::_FALSE || mt==StateMatcher::ANY){
		return 10000;
	}
	else if (mt==StateMatcher::_TRUE){
		return sum();
	}
	else if (mt==StateMatcher::D_NEW){
		return sum_D(Dn);
	}
	else if (mt==StateMatcher::POSE){
		return sum_r();
	}
	else if (mt==StateMatcher::D_INIT){
		return sum_D(Di);
	}
	else if (mt==StateMatcher::ABSTRACT){
		return sum_D(Dn) +sum_D(Di);
	}
}

void StateDifference::init(const State& s1, const State& s2){ //observed, desired
	pose.p.x= s1.endPose.p.x-s2.endPose.p.x; //endpose x
	pose.p.y=s1.endPose.p.y-s2.endPose.p.y; //endpose y
	pose.q.Set(angle_subtract(s1.endPose.q.GetAngle(), s2.endPose.q.GetAngle()));
	// if (s1.Dn.getAffIndex()==NONE && s2.Dn.getAffIndex()==NONE){
	// 	return;
	// }
	if (s1.Dn.getAffIndex()!= s2.Dn.getAffIndex()){
		fill_invalid_bodyfeatures(Dn);
	}
	else{
		fill_valid_bodyfeatures(Dn, s1, s2, DN);
	}
	if (s1.Di.getAffIndex()!=s2.Di.getAffIndex()){
		fill_invalid_bodyfeatures(Di);
	}
	else{
		fill_valid_bodyfeatures(Di, s1, s2, DI);
	}
	
}

void StateDifference::fill_invalid_bodyfeatures(BodyFeatures & bf){
	bf.pose.p.x=10000;
	bf.pose.p.y=10000;
	bf.pose.q.Set(MAX_ANGLE_ERROR);
	bf.halfLength=10000;
	bf.halfWidth=10000;
}

void StateDifference::fill_valid_bodyfeatures(BodyFeatures & bf, const State& s1, const State& s2, WHAT_D_FLAG flag){
	b2Transform p1=b2Transform_zero, p2=b2Transform_zero;
	if (flag==DI){
		if (s1.Di.getAffIndex()==NONE && s2.Di.getAffIndex()==NONE){
			return;
		}
		p1=s1.start_from_Di();
		p2=s2.start_from_Di();
		bf.halfWidth=(s1.Di.bodyFeatures().halfWidth-s2.Di.bodyFeatures().halfWidth);
		bf.halfLength=(s1.Di.bodyFeatures().halfLength-s2.Di.bodyFeatures().halfLength);
	}
	else if (flag==DN){
		if (s1.Dn.getAffIndex()==NONE && s2.Dn.getAffIndex()==NONE){
			return;
		}
		p1=s1.end_from_Dn();
		p2=s2.end_from_Dn();
		bf.halfWidth=(s1.Dn.bodyFeatures().halfWidth-s2.Dn.bodyFeatures().halfWidth);
		bf.halfLength=(s1.Dn.bodyFeatures().halfLength-s2.Dn.bodyFeatures().halfLength);

	}
	bf.pose.p.x= p1.p.x - p2.p.x; //disturbance x
	bf.pose.p.y= p1.p.y - p2.p.y; //disturbance y
	float pose_q= angle_subtract(p1.q.GetAngle(), p2.q.GetAngle());
	bf.pose.q.Set(pose_q);

}


void gt::fill(simResult sr, State* s, Edge* e){
	if (NULL!=s){
		s->Dn = sr.collision;
		s->endPose = sr.endPose;
		s->outcome = sr.resultCode;
		s->filled=true;
	}
	if (NULL!=e){
		e->step = gt::simToMotorStep(sr.step);
	}
}

int gt::simToMotorStep(int simStep){
	float result =std::floor(float(simStep)/(HZ*MOTOR_CALLBACK)+0.5);
	return int(result);
}

int gt::distanceToSimStep(const float& s, const float& ds){
	float time=s/ds; //ds:1=s:time
	int sim_step=(time*HZ);
	return sim_step;
}

// simResult State::getSimResult(){
// 	simResult result;
// 	result.collision= disturbance;
// 	result.endPose=endPose;
// 	//result.step= step;
// 	result.valid = filled;
// 	result.resultCode=outcome;
// 	return result;
// }

void gt::update(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, std::unordered_map<State*, ExecutionError>& errorMap, int it){
	if (e==edgeDescriptor()){
		return;
	}
	ExecutionError result;
	if (!current){
		g[e].step = sk.second.step;
	}
	// else if (g[e].direction==DEFAULT& g[e.m_target].disturbance.isValid()){
	// 	result.setR(g[e.m_target].disturbance.getPosition().x-sk.first.disturbance.getPosition().x);
	// 	errorMap.insert_or_assign(g[e.m_target].ID, result);
	// }
	// else if ((g[e.m_target].direction==LEFT || g[e.m_target].direction==RIGHT )& g[e.m_target].disturbance.isValid()){
	// 	orientation o=subtract(g[e.m_target].disturbance.getOrientation(),sk.first.disturbance.getOrientation());
	// 	result.setTheta(o.second);
	// 	errorMap.insert_or_assign(g[e.m_target].ID, result);
	// }
	g[e.m_target].Dn = sk.first.Dn;
	if(sk.first.label==g[e.m_target].label){
		g[e.m_target].endPose = sk.first.endPose;
	}
	g[e.m_target].options = sk.first.options;
	g[e.m_target].nObs++;
	if (e.m_source!=e.m_target){
		g[e.m_target].start=sk.first.start;
	}
	if (!g[e.m_target].visited()){
		g[e.m_target].phi=sk.first.phi;
	}
	g[e].it_observed=it;
}

void gt::set(edgeDescriptor e, std::pair <State, Edge> sk, TransitionSystem& g, bool current, std::unordered_map<State*, ExecutionError>& errorMap, int it){
	update(e, sk, g, current, errorMap, it);
	g[e.m_target].outcome = sk.first.outcome;
	//g[e.m_target].label=sk.first.label;
}

std::vector <edgeDescriptor> gt::outEdges(TransitionSystem&g, vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::out_edges(v, g);
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei)].direction == d || d==UNDEFINED){
			result.push_back(*ei);
		}
	}
	return result;
}

std::vector <edgeDescriptor> gt::inEdges(TransitionSystem&g, const vertexDescriptor& v, const Direction &d){
	std::vector <edgeDescriptor> result;
	auto es = boost::in_edges(v, g);
	if (v==TransitionSystem::null_vertex()){
		return result;
	}
	for (auto ei = es.first; ei!=es.second; ++ei){
		if (g[(*ei)].direction == d || d==UNDEFINED){
			if ((*ei).m_source!=v){
				result.push_back(*ei);
			}
		}
	}
	return result;
}

std::pair< bool, edgeDescriptor> gt::getMostLikely(TransitionSystem& g, std::vector <edgeDescriptor> oe, int it){
	std::pair< bool, edgeDescriptor> mostLikely(false, edgeDescriptor());
	float prob=-1;
	for (edgeDescriptor e:oe){
		if (g[e].weighted_probability(it)>prob){
			mostLikely.second=e;
			prob=g[e].weighted_probability(it);
		}
	}
	mostLikely.first=!oe.empty();
	return mostLikely;
}


Disturbance gt::getExpectedDisturbance(TransitionSystem& g, vertexDescriptor v, Direction d, int it){
	std::vector<edgeDescriptor> oe=outEdges(g, v, d);
	Disturbance result=Disturbance();
	if (oe.empty()){
		return result;
	}
	std::pair<bool,edgeDescriptor> mostLikely=getMostLikely(g, oe, it);
	if (mostLikely.first){
		result=g[mostLikely.second.m_target].Dn;
	}
	return result;

}
std::pair <bool,edgeDescriptor>  gt::visitedEdge(const std::vector <edgeDescriptor> &es, TransitionSystem& g, vertexDescriptor cv){
	std::pair <bool,edgeDescriptor> result(false, edgeDescriptor());
	for (edgeDescriptor e:es){
		if ((g[e.m_source].visited() & g[e.m_target].visited()) || e.m_source==0 || (e.m_source==cv & cv !=TransitionSystem::null_vertex()) ){
			result.first=true;
			result.second=e;
			return result;
		}
	}
	return result;
}


void gt::adjustProbability(TransitionSystem &g, edgeDescriptor e){
	if (e.m_target==TransitionSystem::null_vertex()){
		return;
	}
	//auto es= boost::out_edges(e.m_source, g);
	std::vector <edgeDescriptor> es=gt::outEdges(g, e.m_source, g[e].direction);
	float totObs=0;
	//std::vector <edgeDescriptor> sameTask;
	//find total observations
	for (edgeDescriptor & ei:es){
		g[ei].probability=g[ei.m_target].nObs/es.size();
		//edgeDescriptor ei_deref=*ei;
		// if (g[ei].direction==g[e].direction){
		 	totObs+=g[ei.m_target].nObs;
		// 	//sameTask.push_back(ei);
		 	//g[*ei].probability=g[e.m_target].nObs/g[e.m_source].nObs;
		// }
	}
	//adjust
	for (edgeDescriptor &ei: es){
		g[ei].probability=g[ei.m_target].nObs/totObs;
	}
}

std::pair <edgeDescriptor, bool> gt::add_edge(const vertexDescriptor & u, const  vertexDescriptor & v, TransitionSystem& g, const int &it, Direction d){
	std::pair <edgeDescriptor, bool> result=boost::edge(u, v, g);
	if (u==v){
		if (d==UNDEFINED){
			return result;
		}
	}
	auto oe=outEdges(g, u, d);
	for (auto e:oe){
		if (g[v].Dn == g[e.m_target].Dn&& u!=v ){

			return result;
		}
	}
	result=boost::add_edge(u, v, g);
	g[result.first].it_observed=it;
	return result;
}

bool gt::check_edge_direction(const std::pair<edgeDescriptor, bool> & ep, TransitionSystem& g, Direction d){
	bool result=false;
	if (ep.second){
		result=g[ep.first].direction==d;
	}
	return result;
}


std::vector <vertexDescriptor> gt::task_vertices( vertexDescriptor v, TransitionSystem& g, const int & it, const vertexDescriptor & current_v, std::pair<bool, edgeDescriptor>* ep){
	std::vector <vertexDescriptor> result= {v};
	Direction d=UNDEFINED;
	std::pair<bool, edgeDescriptor>ep2(false, edgeDescriptor()), _ep=ep2;
	do {
		std::vector <edgeDescriptor> ie=gt::inEdges(g, v);
		ep2= visitedEdge(ie, g,v);
		if (!ep2.first){
			ep2=getMostLikely(g, ie, it);
		}
		if (ep2.first){
			if (ep2.second.m_target==result[0]){ //size 1
				_ep=ep2; //assign ep to define direction
				d= g[_ep.second].direction;
				//if (ie.size()>1){
				if (ep!=NULL){
					g[_ep.second].it_observed=it;
				}
				for (edgeDescriptor e: ie){
					if (g[e].direction==d && e!=ep2.second && g[e.m_source].Di == g[_ep.second.m_source].Di &&g[e.m_source].Dn == g[_ep.second.m_target].Dn){
						ep2.second=e;
						break;
					}
				//}
			}
			}
			else if (g[ep2.second].direction==d &&
			 		g[ep2.second.m_target].Di == g[_ep.second.m_target].Di &&
			 		g[ep2.second.m_target].Dn == g[_ep.second.m_target].Dn){ //same task!
				result.push_back(ep2.second.m_target); //source
			}


		}
		else{
			break;
		}
		v=ep2.second.m_source;
		if (ep2.second.m_target==current_v){ //source
			//result.push_back(0);
			//ep=ep2; //reassign ep so that the source is the vertex from which the task actually started
			break;
		}
	}while(g[ep2.second].direction==d);
	//ep=boost::edge(ep2.second)
	std::reverse(result.begin(), result.end());
	if (NULL!=ep){
		*ep=_ep;
	}
	return result;
}

bool StateMatcher::match_equal(const MATCH_TYPE& candidate, const MATCH_TYPE& desired){
	bool result=false;
	switch (desired){ //the desired match
		case ANY:
			if (candidate!=_FALSE){
				result=true;
			}
			break;
		case POSE:
			if (candidate==_TRUE || candidate == POSE){
				result=true;
			}
			break;
		case ABSTRACT:
			if (candidate==_TRUE || candidate ==ABSTRACT){
				result=true;
			}
			break;
		// case D_POSE:
		// 	if (candidate==_TRUE || candidate==DISTURBANCE || candidate==D_POSE){
		// 		result=true;
		// 	}
		// 	break;
		default:
			result =int(candidate)==int(desired);
		break;
	}
	return result;
}

// StateDifference StateMatcher::get_state_difference(State s1, State s2){
// 	StateDifference result;
// 	result.D_position.x= s1.disturbance.getPosition().x - s2.disturbance.getPosition().x; //disturbance x
// 	result.D_position.y= s1.disturbance.getPosition().y - s2.disturbance.getPosition().y; //disturbance y
// 	result.D_type= s1.disturbance.getAffIndex()-s2.disturbance.getAffIndex(); //disturbance type
// 	result.r_position.x= s1.endPose.p.x-s2.endPose.p.x; //endpose x
// 	result.r_position.y=s1.endPose.p.y-s2.endPose.p.y; //endpose y
// 	//adjusting for angles with different signs close to pi
// 	// float candidate_angle=s2.endPose.q.GetAngle();
// 	// if (fabs(s1.endPose.q.GetAngle())> 3*M_PI_4 || fabs(candidate_angle)> 3*M_PI_4){
// 	// 	if (s1.endPose.q.GetAngle()<0 & candidate_angle>0){
// 	// 		candidate_angle-=2*M_PI;
// 	// 	}
// 	// 	else if(candidate_angle<0 & s1.endPose.q.GetAngle()>0){
// 	// 		candidate_angle+=2*M_PI;
// 	// 	}
// 	// }
// 	// result.ppse.q.=s1.endPose.q.GetAngle()-candidate_angle;
// 	result.r_angle= get_angle_difference(s1.endPose.q.GetAngle(), s2.endPose.q.GetAngle());
// 	result.D_angle=get_angle_difference(s1.disturbance.pose().q.GetAngle(), s2.disturbance.pose().q.GetAngle());
// 	result.D_width=(s1.disturbance.bodyFeatures().halfWidth-s2.disturbance.bodyFeatures().halfWidth)*2;
// 	result.D_length=(s1.disturbance.bodyFeatures().halfLength-s2.disturbance.bodyFeatures().halfLength)*2;
// 	return result;
// }


// float StateMatcher::sumVector(DistanceVector vec){
// 	float result=0;
// 	for (float i:vec){
// 		result+=abs(i);
// 	}
// 	return result;
// }



StateMatcher::MATCH_TYPE StateMatcher::isMatch(StateDifference sd, float endDistance){
	float coefficient=get_coefficient(endDistance);
	StateMatcher::StateMatch match(sd, error, coefficient);
    return match.what();
}

StateMatcher::MATCH_TYPE StateMatcher::isMatch(const State & s, const State &candidate, const State *src, StateDifference*_sd,bool match_outcome){
	//src is the source of candidate
	StateDifference sd(s, candidate);
	float stray=0;
	// if (src!=NULL && s.label!=UNDEFINED){
	// 	b2Vec2 stray_v;
	// 	stray_v.x=s.endPose.p.x-src->endPose.p.x;
	// 	stray_v.y=s.endPose.p.y-src->endPose.p.y;
	// 	stray=(stray_v).Length();
	// }
	if ((stray>error.endPosition && s.label==candidate.label)){ //
		sd.pose.p.x=10000; // now pose will not be matched
		sd.pose.p.y=10000;
		sd.pose.q.Set(MAX_ANGLE_ERROR);
	}
	if (NULL!=_sd){
		*_sd=sd;
	}
    return isMatch(sd, s.endPose.p.Length()) ;
}


std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> StateMatcher::match_vertex(TransitionSystem g, vertexDescriptor src, Direction d, State s, StateMatcher::MATCH_TYPE mt){
    std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex());
	auto edges= boost::out_edges(src, g);
	for (auto ei=edges.first; ei!=edges.second; ++ei){
		//MATCH_TYPE match = isMatch(s, g[ei.dereference().m_source]);
		MATCH_TYPE match = isMatch(s, g[ei.dereference().m_target]);
		if (g[(*ei)].direction && match_equal(match, mt)){
			result.first=match;
			result.second=(*ei).m_target;
			break;
		}
	}
    return result;
}

float StateMatcher::get_coefficient(const float & endDistance){
	float coefficient=1.0;
	// if (endDistance>COEFFICIENT_INCREASE_THRESHOLD){
	// 	float scale=1+(endDistance-COEFFICIENT_INCREASE_THRESHOLD);// /.9
	// 	//coefficient+=(endDistance-COEFFICIENT_INCREASE_THRESHOLD)/2;
	// 	coefficient*=scale; //it's a bit high but need for debugging (before *1.2)
	// }
	return coefficient;
}


// void StateMatcher::ICOadjustWeight(DistanceVector E, DistanceVector dE){
// 	for (int i=0; i<weights.size();i++){
// 		//float weight= weights[i];
// 		weights[i]+=mu*E[i]*dE[i];
// 	}
// }

// std::pair <bool, vertexDescriptor> StateMatcher::soft_match(TransitionSystem& g, b2Transform pose){
// 	std::pair <bool, vertexDescriptor> result;
// 	auto es= boost::edges(g);
// 	for (auto ei=es.first; ei!=es.second; ei++){
// 		float x=g[(*ei).m_target].endPose.p.x- pose.p.x;
// 		float y=g[(*ei).m_target].endPose.p.y- pose.p.y;
// 		float theta=g[(*ei).m_target].endPose.q.GetAngle()- pose.q.GetAngle();

// 		if (fabs(x)<error.endPosition & fabs(y)<error.endPosition & fabs(theta)<error.angle){

// 		}
// 	}
// 	return result;
// }


bool operator!=(Transform const &t1, Transform const& t2){
	return t1.p.x != t2.p.x || t1.p.y != t2.p.y || t1.q.GetAngle() != t2.q.GetAngle();
}

bool operator==(Transform const &t1, Transform const& t2){
	return (t1.p.x == t2.p.x) && (t1.p.y == t2.p.y) && (t1.q.GetAngle() == t2.q.GetAngle());
}

void operator-=(Transform & t1, Transform const&t2){
	t1.p.x-=t2.p.x;
	t1.p.y-=t2.p.y;
	t1.q.Set(angle_subtract(t1.q.GetAngle(), t2.q.GetAngle()));
}

void operator+=(Transform & t1, Transform const&t2){
	t1.p.x+=t2.p.x;
	t1.p.y+=t2.p.y;
	t1.q.Set(t1.q.GetAngle()+t2.q.GetAngle());
}

Transform operator+(Transform const & t1, Transform const&t2){
	b2Transform result;
	result.p.x=t1.p.x+t2.p.x;
	result.p.y=t1.p.y+t2.p.y;
	result.q.Set(t1.q.GetAngle()+t2.q.GetAngle());
	return result;
}

Transform operator-(Transform const & t1, Transform const&t2){
	b2Transform result;
	result.p.x=t1.p.x-t2.p.x;
	result.p.y=t1.p.y-t2.p.y;
	result.q.Set(angle_subtract(t1.q.GetAngle(), t2.q.GetAngle()));
	return result;

}

Transform operator-(Transform const & t){
	b2Transform result;
	result.p.x=-(t.p.x);
	result.p.y=-(t.p.y);
	result.q.Set(-t.q.GetAngle());
	return result;

}

