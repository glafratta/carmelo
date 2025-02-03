#ifndef GENERAL_H
#define GENERAL_H
#include <set>
//#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp> //LMEDS
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <map>
#include <boost/property_map/property_map.hpp> //property map
#include <boost/graph/subgraph.hpp>
//#include <boost/variant/get.hpp> //get function
#include <boost/graph/copy.hpp>
#include <utility>
#include "disturbance.h"

const float NAIVE_PHI=10.0;
// enum M_CODES {THREE_M=3, FOUR_M=4};

// enum GRAPH_CONSTRUCTION {BACKTRACKING, A_STAR, A_STAR_DEMAND, E};
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
		return phi<NAIVE_PHI;
	}

	void resetVisited(){
		phi=NAIVE_PHI;
	}

	b2Transform start_from_Di()const;

	b2Transform end_from_Dn()const;

	float distance();

};




struct StateDifference{
	// b2Vec2 r_position=b2Vec2(0,0);
	// float r_angle=0;
	b2Transform pose=b2Transform_zero;
	BodyFeatures Di, Dn;
	// int D_type=0;
	// b2Vec2 D_position=b2Vec2(0,0);
	// float D_angle=0;
	// float D_width=0;
	// float D_length=0;
	enum WHAT_D_FLAG{DI, DN};

	StateDifference()=default;

	StateDifference(const State& s1, const State& s2){
		init(s1, s2);
	}

	float sum(){
		return sum_r()+sum_D(Di)+ sum_D(Dn);
	}

	float sum_r(){
		return fabs(pose.p.x)+fabs(pose.p.y)+fabs(pose.q.GetAngle());
	}

	float sum_D_pos(const BodyFeatures& bf){
		return fabs(bf.pose.p.x)+fabs(bf.pose.p.y)+bf.pose.q.GetAngle();
	}

	float sum_D_shape(const BodyFeatures& bf){
		return fabs(bf.width())+fabs(bf.length());
	}

	float sum_D(const BodyFeatures& bf){
		return sum_D_pos(bf)+sum_D_shape(bf);
	}
	
	float get_sum(int);

	void init(const State& ,const State&);

	void fill_invalid_bodyfeatures(BodyFeatures &);

	void fill_valid_bodyfeatures(BodyFeatures &, const State&, const State&, WHAT_D_FLAG);
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
typedef boost::subgraph<TransitionSystem> Subgraph;
typedef boost::graph_traits<TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<TransitionSystem>::edge_iterator edgeIterator;

namespace math {
	void applyAffineTrans(const b2Transform& deltaPose, b2Transform& pose);

	void applyAffineTrans(const b2Transform&, State& );

	void applyAffineTrans(const b2Transform& , Task* );

	void applyAffineTrans(const b2Transform&, TransitionSystem&);

	void applyAffineTrans(const b2Transform&, Disturbance&);

};


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

//	NotSelfEdge(TransitionSystem * _g):g(_g){}

	bool operator()(const edgeDescriptor & e) const {
		bool not_self= e.m_source!=e.m_target;
	}
// private:
// TransitionSystem * g=NULL;
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
	//std::set <edgeDescriptor> forget;
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

	std::pair <edgeDescriptor, bool> add_edge(const vertexDescriptor&, const vertexDescriptor &, TransitionSystem&, const int &, Direction d=UNDEFINED); //wrapper around boost function, disallows edges to self

	bool check_edge_direction(const std::pair<edgeDescriptor, bool> &, TransitionSystem&, Direction);

	std::vector <vertexDescriptor> task_vertices(vertexDescriptor, TransitionSystem&, const int &, const vertexDescriptor &, std::pair<bool, edgeDescriptor>&);
}



typedef boost::filtered_graph<TransitionSystem, NotSelfEdge, Connected> FilteredTS;
typedef boost::filtered_graph<TransitionSystem, boost::keep_all, Visited> VisitedTS;


class StateMatcher{
	public:
		enum MATCH_TYPE {_FALSE=0, D_NEW=2, DN_POSE=3, _TRUE=1, ANY=4, D_INIT=5, ABSTRACT=6, DI_POSE=7, DN_SHAPE=8, DI_SHAPE=9, POSE=10};
        //std::vector <float> weights; //disturbance, position vector, angle
		//assume mean difference 0
		//std::vector <float> SDvector={0.03, 0.03, 0, 0.08, 0.08, M_PI/6};//hard-coded standard deviations for matching

		struct Error{
			const float endPosition=0.05;//0.05;
			const float angle= M_PI/6;
			const float dPosition= 0.065;//0.065; 
			const float affordance =0;
			const float D_dimensions=D_DIMENSIONS_MARGIN;
		}error;

		float mu=0.001;
	    StateMatcher()=default;
		// void initOnes(){
		// 	for (auto i=weights.begin(); i!= weights.end(); i++){
		// 		*i=1.0;
		// 	}
		// }

	//StateDifference get_state_difference(State, State );

		struct StateMatch{

			bool exact(){
				return pose() && abstract();
			}

			bool pose(){
				return position && angle;
			}

			bool abstract(){  //states match Di and Dn regardless of objective pose
				return Di_exact() && Dn_exact();
			}

			bool Di_exact(){
				return Di_position && Di_angle && Di_shape;
			}

			bool Dn_exact(){
				return Dn_position && Dn_angle &&  Dn_shape;
			}

			bool Di_pose(){
				return Di_position &&  Di_angle;
			}

			bool Dn_pose(){
				return Dn_position && Dn_angle;
			}

			bool shape_Di(){
				return  Di_shape;
			}

			bool shape_Dn(){
				return Dn_shape;
			}
			bool Dn(){
				return Dn_shape && Dn_pose();
			}

			bool Di(){
				return Di_shape && Di_pose();
			}

			StateMatch() =default;

			StateMatch(const StateDifference& sd, StateMatcher::Error error, float coefficient=1){
				position = sd.pose.p.Length()<(error.endPosition*coefficient);
				angle=fabs(sd.pose.q.GetAngle())<error.angle;
				//d_type=sd.D_type==0;
				Dn_position= sd.Dn.pose.p.Length()<(error.dPosition*coefficient);
				Di_position= sd.Di.pose.p.Length()<(error.dPosition*coefficient);
				Dn_angle=fabs(sd.Dn.pose.q.GetAngle())<error.angle;
				Di_angle=fabs(sd.Di.pose.q.GetAngle())<error.angle;
				bool Dn_below_threshold_w=fabs(sd.Dn.width())<(error.D_dimensions*coefficient*2);
				bool Dn_below_threshold_l=fabs(sd.Dn.length())<(error.D_dimensions*coefficient*2);
				bool Di_below_threshold_w=fabs(sd.Di.width())<(error.D_dimensions*coefficient*2);
				bool Di_below_threshold_l=fabs(sd.Di.length())<(error.D_dimensions*coefficient*2);
				Dn_shape= Dn_below_threshold_l && Dn_below_threshold_w;
				Di_shape= Di_below_threshold_l && Di_below_threshold_w;

			}

			StateMatcher::MATCH_TYPE what(){
				if (exact()){ //match position and disturbance
					return _TRUE;
				}
				else if (abstract()){
					return ABSTRACT;
				}
				else if (Dn_exact()){
					return D_NEW;
				}
				else if (Dn_pose()){
					return DN_POSE;
				}
				else if (shape_Dn()){
					return DN_SHAPE;
				}
				else if (Di_exact()){
					return D_INIT;
				}
				else if (Di_pose()){
					return DI_POSE;
				}
				else if (shape_Di()){
					return DI_SHAPE;
				}
				else{
					return _FALSE;
				}
			}

			private:

			bool position=false;
			bool angle=false;
			bool Di_position=false;
			bool Di_angle=false;
			bool Di_shape=false;
			//bool Di_type=0;
			bool Dn_position=false;
			bool Dn_angle=false;
			bool Dn_shape=false;
			//bool Dn_type=0;
		};

		bool match_equal(const MATCH_TYPE& candidate, const MATCH_TYPE& desired);

		MATCH_TYPE isMatch(StateDifference, float endDistance=0); //endDistance=endpose

		MATCH_TYPE isMatch(const State &, const State&, const State* src=NULL, StateDifference * _sd=NULL, bool match_outcome=false); //first state: state to find a match for, second state: candidate match

		std::pair<MATCH_TYPE, vertexDescriptor> match_vertex(TransitionSystem, vertexDescriptor, Direction, State, StateMatcher::MATCH_TYPE mt=StateMatcher::_TRUE); //find match amoung vertex out edges

		float get_coefficient(const float &);
	private:


	const float COEFFICIENT_INCREASE_THRESHOLD=0.0;
};

template <class I>
auto check_vector_for(std::vector <I>& vector, const I& item){
	for (int i=0; i<vector.size(); i++){
		if (vector[i]==item){
			return vector.begin()+i;
		}
	}
	return vector.end();
}
#endif