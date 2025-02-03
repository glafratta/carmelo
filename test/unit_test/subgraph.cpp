#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>

namespace cm{
    struct State{
        int i=0;
    };

    struct Edge{
        int j=0;
    };  

    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::bidirectionalS, cm::State, cm::Edge> TransitionSystem;
}




// #ifndef CUSTOM_PROPERTY_MAPS //works
// typedef boost::property<boost::edge_index_t, int> Edge;
// #else
// struct Edge{
//     int j=0;
// };
// #endif


typedef boost::subgraph<cm::TransitionSystem> CM;
typedef boost::graph_traits<cm::TransitionSystem>::vertex_iterator vertexIterator; 
typedef boost::graph_traits<cm::TransitionSystem>::vertex_descriptor vertexDescriptor;
typedef boost::graph_traits<cm::TransitionSystem>::edge_descriptor edgeDescriptor;
typedef boost::graph_traits<cm::TransitionSystem>::edge_iterator edgeIterator;

template <>
struct boost::property_map<cm::TransitionSystem, boost::edge_index_t>
    : boost::property_map<cm::TransitionSystem, decltype(&cm::Edge::j)> {};


namespace cm{
    static inline auto get(boost::edge_index_t, cm::TransitionSystem const& g) { return boost::get(&Edge::j, g); }
    static inline auto get(boost::edge_index_t, TransitionSystem& g) { return boost::get(&Edge::j, g); }

}


int main(){
    CM g;
    vertexDescriptor v0 = boost::add_vertex(g);
    vertexDescriptor v1 = boost::add_vertex(g);
    vertexDescriptor v2 = boost::add_vertex(g);
    vertexDescriptor v3 = boost::add_vertex(g);
    vertexDescriptor v4 = boost::add_vertex(g);
    boost::add_edge(v0, v1, g);
    boost::add_edge(v1, v2, g);
    boost::add_edge(v2, v3, g);
    boost::add_edge(v2, v4, g);
        CM g1=g.create_subgraph();
  //  boost::add_vertex(v1, g1);
    //boost::add_vertex(v2, g1);
    //boost::add_vertex(v3, g1);


}