#ifndef GENERAL_H
#define GENERAL_H
#include <set>
#include <opencv2/core.hpp>
// #include <opencv2/calib3d.hpp> //LMEDS
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <map>
#include <boost/property_map/property_map.hpp> //property map
#include <boost/graph/copy.hpp>
#include <utility>
#include "disturbance.h"

const float NAIVE_PHI=10.0;
class Task;
enum VERTEX_LABEL {UNLABELED, MOVING, ESCAPE, ESCAPE2};

float angle_subtract(float a1, float a2);

struct ComparePair{
	ComparePair()=default;

	template <class V>
	bool operator()(const std::pair<V, float> & p1, const std::pair<V, float> &p2) const{
		return p1.second<p2.second;
	}
};

struct Edge{
	Direction direction=DEFAULT;
	float probability=1.0;
	int step=0;
	int it_observed=-1; //last iteration where this edge was observed

	Edge()=default;

	Edge(Direction d):direction(d){}

	float weighted_probability(int it){
		float result=0;
		if (it_observed>=0){
			result=probability*float(it_observed)/float(it);
		}
		return result;
	}
};


struct State{
	Disturbance Di; //initial Disturbance
	Disturbance Dn; //new Disturbance
	b2Transform endPose = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)), start = b2Transform(b2Vec2(0.0, 0.0), b2Rot(0)); 
	simResult::resultType outcome;
	std::vector <Direction> options;
	int nodesInSameSpot =0;
	bool filled =0;
	int nObs=0;
	State* ID=this;
	float phi=NAIVE_PHI; //arbitrarily large phi
	VERTEX_LABEL label=VERTEX_LABEL::UNLABELED;

	
	State()=default;

	State(const b2Transform &_start): start(_start){}

	State(const b2Transform &_start, const Disturbance& di): start(_start), Di(di){}

	bool visited(){
		return phi<=NAIVE_PHI;
	}

	void resetVisited(){
		phi=NAIVE_PHI;
	}

	b2Transform from_disturbance()const;

	float distance();

};

namespace math {
	void applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose);

	void applyAffineTrans(const b2Transform&, State& );
};



struct StateDifference{
	b2Vec2 r_position=b2Vec2(0,0);
	float r_angle=0;
	int D_type=0;
	b2Vec2 D_position=b2Vec2(0,0);
	float D_angle=0;
	float D_width=0;
	float D_length=0;

	StateDifference()=default;

	StateDifference(const State& s1, const State& s2){
		init(s1, s2);
	}

	float sum(){
		return sum_r()+sum_d_pos()+sum_d_shape();
	}

	float sum_r(){
		return fabs(r_position.x)+fabs(r_position.y)+fabs(r_angle);
	}

	float sum_d_pos(){
		return fabs(D_position.x)+fabs(D_position.y)+D_angle;
	}

	float sum_d_shape(){
		return fabs(D_width)+fabs(D_length);
	}

	float sum_d(){
		return sum_d_pos()+sum_d_shape();
	}
	
	float get_sum(int);

	void init(const State& s1,const State& s2);

};


typedef b2Transform Transform;
bool operator!=(Transform const &, Transform const &);
bool operator==(Transform const &, Transform const &);
void operator-=(Transform &, Transform const&);
void operator+=(Transform &, Transform const&);
Transform operator+( Transform const &, Transform const &);
Transform operator-( Transform const &, Transform const &);
Transform operator-(Transform const &);


typedef std::pair<bool, float> orientation;
orientation subtract(orientation, orientation);

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, State, Edge> TransitionSystem;
typedef boost::graph_traits<TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_iterator edgeIterator;

struct Connected{
	Connected(){}
	Connected(TransitionSystem * ts): g(ts){}
	
	bool operator()(const vertexDescriptor& v)const{
	 	bool in= boost::in_degree(v, *g)>0;
		bool out =boost::out_degree(v, *g)>0;
	 	return in || out;
	}
private:
TransitionSystem * g;
};

struct MoreLikely{
	bool operator()(Edge e1, Edge e2){//const
		return e1.probability >e2.probability;
	}
};

struct NotSelfEdge{
	NotSelfEdge(){}


	bool operator()(const edgeDescriptor & e) const {
		bool not_self= e.m_source!=e.m_target;
	}
};

struct Remember{
	Remember(){}
	Remember(TransitionSystem* ts):g(ts){}

	bool operator()(const edgeDescriptor& e){//const
		if ((*g)[e].probability<FORGET_THRESHOLD){ //filter signal
		 	return false;
		 }
		return true;
	}

	private: 
	TransitionSystem *g;
};

struct Visited{ //for debug
	Visited(){}
	Visited(TransitionSystem * ts):g(ts){}

	bool operator()(const vertexDescriptor&v)const{
		return (*g)[v].visited();
	}
	private:
	TransitionSystem *g;
};

struct is_not_v{
	is_not_v(){}
	is_not_v(vertexDescriptor _cv): cv(_cv){}
	bool operator()(edgeDescriptor e){
		return e.m_target!=cv;
	}	

	private:
	vertexDescriptor cv;
};

struct ExecutionError{

	ExecutionError(){}

	ExecutionError(float fr, float ft){
		_r=fr;
		_theta=ft;
	}

	float r(){
		return _r;
	}

	float theta(){
		return _theta;
	}

	void setTheta(float f){
		_theta=f;
	}

	void setR(float f){
		_r=f;
	}
	private:
	float _r=0;
	float _theta=0;
};

typedef std::pair<vertexDescriptor, std::vector<vertexDescriptor>> Frontier;

struct ComparePhi{

	ComparePhi(){}

	bool operator()(const std::pair<State*, Frontier>& p1, const std::pair<State*, Frontier>& p2) const{
		return (*p1.first).phi<(*p2.first).phi;
	}
};


namespace gt{

	void fill(simResult, State* s=NULL, Edge* e=NULL);

	int simToMotorStep(int);

	int distanceToSimStep(const float&, const float&);
	
	void update(edgeDescriptor,  std::pair <State, Edge>, TransitionSystem&, bool, std::unordered_map<State*, ExecutionError>&, int); //returns disturbance rror based on expected vs observed D

	void set(edgeDescriptor,  std::pair <State, Edge>, TransitionSystem&, bool, std::unordered_map<State*, ExecutionError>&, int);

	std::pair< bool, edgeDescriptor> getMostLikely(TransitionSystem&,std::vector<edgeDescriptor>, int);

	std::vector <edgeDescriptor> outEdges(TransitionSystem&, vertexDescriptor, Direction); //returns a vector containing all the out-edges of a vertex which have the specified direction

	std::vector <edgeDescriptor> inEdges(TransitionSystem&, const vertexDescriptor&, const Direction & d = UNDEFINED); //returns a vector containing all the in-edges of a vertex which have the specified direction

	Disturbance getExpectedDisturbance(TransitionSystem&, vertexDescriptor, Direction, int);

	std::pair <bool,edgeDescriptor> visitedEdge(const std::vector <edgeDescriptor>&, TransitionSystem&, vertexDescriptor cv=TransitionSystem::null_vertex());

	void adjustProbability(TransitionSystem&, edgeDescriptor);

	std::pair <edgeDescriptor, bool> add_edge(vertexDescriptor, vertexDescriptor, TransitionSystem&, int); //wrapper around boost function, disallows edges to self

}



typedef boost::filtered_graph<TransitionSystem, NotSelfEdge, Connected> FilteredTS;
typedef boost::filtered_graph<TransitionSystem, boost::keep_all, Visited> VisitedTS;


class StateMatcher{
	public:
		enum MATCH_TYPE {_FALSE=0, DISTURBANCE=2, POSE=3, _TRUE=1, ANY=4, D_POSE=5, D_SHAPE=6};
		struct Error{
			const float endPosition=0.05;//0.05;
			const float angle= M_PI/6;
			const float dPosition= 0.065;//0.065; 
			const float affordance =0;
			const float D_dimensions=D_DIMENSIONS_MARGIN;
		}error;

		float mu=0.001;
	    StateMatcher()=default;

		struct StateMatch{

			bool exact(){
				return pose() && disturbance_exact();
			}

			bool pose(){
				return r_position && r_angle;
			}

			bool disturbance_exact(){
				return d_position && d_angle && d_type && d_shape;
			}

			bool disturbance_pose(){
				return d_position && d_type && d_angle;
			}

			bool disturbance_shape(){
				return d_type && d_shape;
			}

			StateMatch(const StateDifference& sd, StateMatcher::Error error, float coefficient=1){
				r_position = sd.r_position.Length()<(error.endPosition*coefficient);
				r_angle=fabs(sd.r_angle)<error.angle;
				d_type=sd.D_type==0;
				d_position= sd.D_position.Length()<(error.dPosition*coefficient);
				d_angle=fabs(sd.D_angle)<error.angle;
				bool below_threshold_w=fabs(sd.D_width)<(error.D_dimensions*coefficient);
				bool below_threshold_l=fabs(sd.D_length)<(error.D_dimensions*coefficient);
				d_shape= below_threshold_l && below_threshold_w;

			}

			StateMatcher::MATCH_TYPE what(){
				if (exact()){ //match position and disturbance
					return _TRUE;
				}
				else if (pose()){
					return POSE;
				}
				else if (disturbance_exact()){
					return DISTURBANCE;
				}
				else if (disturbance_pose()){
					return D_POSE;
				}
				else if (disturbance_shape()){
					return D_SHAPE;
				}
				else{
					return _FALSE;
				}
			}

			private:
			bool r_position=0;
			bool r_angle=0;
			bool d_position=0;
			bool d_angle=0;
			bool d_shape=0;
			bool d_type=0;
		};
		

		bool match_equal(const MATCH_TYPE&, const MATCH_TYPE&);

		MATCH_TYPE isMatch(StateDifference, float endDistance=0); //endDistance=endpose

		MATCH_TYPE isMatch(const State &, const State&, const State* src=NULL, StateDifference * _sd=NULL); //first state: state to find a match for, second state: candidate match

		std::pair<MATCH_TYPE, vertexDescriptor> match_vertex(TransitionSystem, vertexDescriptor, Direction, State, StateMatcher::MATCH_TYPE mt=StateMatcher::_TRUE); //find match amoung vertex out edges
		

		float get_coefficient(const float &);
	private:


	const float COEFFICIENT_INCREASE_THRESHOLD=0.0;
};

template <class I>
bool check_vector_for(const std::vector <I>& vector, const I& item){
	for (I _item:vector){
		if (_item==item){
			return true;
		}
	}
	return false;
}
#endif