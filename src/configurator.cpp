#include "configurator.h"
#include <chrono>



void ConfiguratorInterface::setReady(bool b){
	ready = b;
}


bool ConfiguratorInterface::isReady(){
	return ready;
}

void Configurator::dummy_vertex(vertexDescriptor src){
	vertexDescriptor prev_current=currentVertex;
	currentVertex=boost::add_vertex(transitionSystem);
	gt::fill(simResult(), &transitionSystem[currentVertex]);
	transitionSystem[currentVertex].nObs++;
	transitionSystem[currentVertex].Di=controlGoal.disturbance;
	currentTask=Task(Direction::STOP);
	movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
	currentEdge = boost::add_edge(src, currentVertex, transitionSystem).first;
	transitionSystem[movingEdge].direction=STOP;
	transitionSystem[currentEdge].direction=STOP;
	errorMap.emplace((transitionSystem[currentVertex].ID), ExecutionError());

}

std::pair <edgeDescriptor, bool> Configurator::add_vertex_now(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Disturbance obs,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, g, edge, topDown);
	if (!g[v1].filled){
		g[v1].Di= obs;
	}
	return result;
}

std::pair <edgeDescriptor, bool> Configurator::add_vertex_retro(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Disturbance obs,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, g, edge, topDown);
	g[v1].Di= g[src].Di;
	g[v1].Dn=g[src].Dn;
	return result;
}



bool Configurator::Spawner(){ 
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++;
	worldBuilder.iteration++;

	sprintf(bodyFile, "/tmp/bodies%04i.txt", iteration);
	sprintf(worldBuilder.bodyFile, "%s",bodyFile);
	FILE *f;
	if (debugOn){
		f = fopen(bodyFile, "w");
		fclose(f);
	}
	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	timeElapsed=float(diff.count())/1000; //express in seconds
	previousTimeScan=now; //update the time of sampling

	if (timerOff){
		timeElapsed = .2;
	}

	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];

	auto startTime =std::chrono::high_resolution_clock::now();
	bool explored=0;
	if (planning){ //|| !planError.m_vertices.empty())
		transitionSystem[movingVertex].Dn=transitionSystem[currentVertex].Dn;
		transitionSystem[movingVertex].Dn.invalidate();
		vertexDescriptor src; //ve=TransitionSystem::null_vertex(),
		if (!planVertices.empty()){
			src=movingVertex;
		}
		else {
			src=currentVertex;
		}
	if (planVertices.empty() && currentTask.motorStep==0){	// boost::out_degree(src, transitionSystem) <1		
		is_not_v not_cv(currentVertex);
		planVertices.clear();
		boost::clear_vertex(movingVertex, transitionSystem);
		if (transitionSystem.m_vertices.size()==1){
			dummy_vertex(currentVertex);//currentEdge.m_source
		}
		currentTask.change=1;
		src=currentVertex;
		resetPhi(transitionSystem);
		explorer(src, transitionSystem, currentTask, world);
		Connected connected(&transitionSystem);
		FilteredTS fts(transitionSystem, NotSelfEdge(), connected); //boost::keep_all()
		TransitionSystem tmp;
		boost::copy_graph(fts, tmp);
		transitionSystem.clear();
		transitionSystem.swap(tmp);		
		planVertices= planner(transitionSystem, src);
		boost::remove_out_edge_if(movingVertex, not_cv, transitionSystem);
		explored=1;
	}
	}
	else {
		if (transitionSystem.m_vertices.size()==1 && iteration<=1){
			movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
			transitionSystem[movingEdge].direction=DEFAULT;
			currentTask.action.init(transitionSystem[movingEdge].direction);
		}
		if (currentTask.action.getOmega()!=0 && currentTask.motorStep<(transitionSystem[movingEdge].step)){
			return 1;
		}
		float _simulationStep=simulationStep;
		adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
		worldBuilder.buildWorld(world, data2fp, transitionSystem[movingVertex].start, currentTask.direction); //was g[v].endPose
		simResult result = simulate(transitionSystem[currentVertex],transitionSystem[currentVertex],currentTask, world, _simulationStep);
		gt::fill(result, transitionSystem[currentVertex].ID, &transitionSystem[currentEdge]);
		currentTask.change = transitionSystem[currentVertex].outcome!=simResult::successful;
	}
	float duration=0;
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
 	duration=abs(float(d.count())/1000); //express in seconds
	if (benchmark){
		FILE * f = fopen(statFile, "a+");
		if (explored){
			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, planVertices, currentVertex);
			fprintf(f, "*");
		}
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), transitionSystem.m_vertices.size(), duration);
		fclose(f);
	}
	worldBuilder.resetBodies();
	return 1;
}

void Configurator::resetPhi(TransitionSystem&g){
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		g[*vi].resetVisited();
	}
}



std::pair <bool, Direction> Configurator::getOppositeDirection(Direction d){
	std::pair <bool, Direction> result(false, DEFAULT);
		switch (d){
		case Direction::LEFT: result.first = true; result.second = RIGHT;break;
		case Direction::RIGHT: result.first = true; result.second = LEFT;break;
		default:
		break;
	}
	return result;
}
Disturbance Configurator::getDisturbance(TransitionSystem&g, const  vertexDescriptor& v, b2World & world, const Direction& dir){
	std::vector <edgeDescriptor> oe=gt::outEdges(g, v, DEFAULT);
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=gt::inEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);
		if (visited.first){
			if (oe.empty()){
				if (g[v].Di.isValid() && g[v].Di.affordanceIndex==AVOID && g[visited.second].direction!=dir){
					//potentially if dir=DEF start from g[v].start
					Task task(g[v].Di, DEFAULT, g[v].endPose, true);
					Robot robot(&world);
					robot.body->SetTransform(task.start.p, task.start.q.GetAngle());
					b2Body * d=worldBuilder.makeBody(world, g[v].Di.bf);
					b2AABB box =worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
					b2Fixture *sensor =GetSensor(robot.body);
					bool overlap=overlaps(robot.body, d) && sensor;
					worldBuilder.world_cleanup(&world);
					if (overlap){
						return g[v].Di;
					}
				}
				//check if Di was eliminated 
				return controlGoal.disturbance;
			}
			else {
				std::pair< bool, edgeDescriptor> e=gt::getMostLikely(g, oe, iteration);
				return g[e.second.m_target].Dn; //IS THIS GOING TO GIVE ME PROBLEMS
			}
		}
	}
	return g[v].Dn;
}


simResult Configurator::simulate(State& state, State src, Task  t, b2World & w, float _simulationStep){
		//EVALUATE NODE()
	simResult result;
	float distance=BOX2DRANGE;
	if (controlGoal.disturbance.isValid()){
		distance= controlGoal.disturbance.getPosition().Length();
	}
	float remaining=distance/controlGoal.action.getLinearSpeed();
	Robot robot(&w);
	worldBuilder.bodies++;
	robot.body->SetTransform(t.start.p, t.start.q.GetAngle());
	b2AABB sensor_aabb=worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
	result =t.willCollide(w, iteration, robot.body, debugOn, remaining, _simulationStep); //default start from 0
	if (b2Vec2(result.endPose.p -src.endPose.p).Length() <=.01){ //CYCLE PREVENTING HEURISTICS
		state.nodesInSameSpot = src.nodesInSameSpot+1; //keep track of how many times the robot is spinning on the spot
	}
	else{
		state.nodesInSameSpot =0; //reset if robot is moving
	}
	return result;
	}




std::vector <std::pair<vertexDescriptor, vertexDescriptor>>Configurator::explorer(vertexDescriptor v, TransitionSystem& g, Task t, b2World & w){
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	Direction direction=currentTask.direction;
	std::vector <vertexDescriptor> priorityQueue = {v}, evaluationQueue;
	std::set <vertexDescriptor> closed;
	b2Transform start= b2Transform(b2Vec2(0,0), b2Rot(0));
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toRemove;
	do{
		v=bestNext;
		closed.emplace(*priorityQueue.begin().base());
		priorityQueue.erase(priorityQueue.begin());
		EndedResult er = controlGoal.checkEnded(g[v], t.direction);
		applyTransitionMatrix(g, v, direction, er.ended, v);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0_exp=v;
			std::vector <Direction> options=g[v0_exp].options;
			while (!options.empty()){
				options.erase(options.begin());
				v0=v0_exp; //node being expanded
				v1 =v0; //frontier
				std::vector <vertexDescriptor> propagated;
				do {
				changeStart(start, v0, g);
				Disturbance Di=getDisturbance(g, v0, w, g[v0].options[0]);
				std::pair <State, Edge> sk(State(start, Di), Edge(g[v0].options[0]));
				t = Task(sk.first.Di, g[v0].options[0], start, true);
				float _simulationStep=BOX2DRANGE;
				adjustStepDistance(v0, g, &t, _simulationStep);
				worldBuilder.buildWorld(w, data2fp, t.start, t.direction, t.disturbance, 0.1, WorldBuilder::PARTITION); //was g[v].endPose
				simResult sim=simulate(sk.first, g[v0], t, w, _simulationStep);
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.direction=t.direction;
				er  = estimateCost(sk.first, g[v0].endPose, sk.second.direction);
				State * source=NULL;
				StateMatcher::MATCH_TYPE vm= matcher.isMatch(g[v], g[currentEdge.m_source]); //see if we are at the beginning of the exploration:
																					//v=0 and currentEdge =src will match so we try to prevent
																			//changing the movign vertex which is by default the origin
				if (v0==movingVertex & vm==StateMatcher::_TRUE){
					source= g[currentEdge.m_source].ID;
				}
				else{
					source=g[v0].ID;
				}
				std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(sk.first, g, source, t.direction);			
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false);
				if (match.first==StateMatcher::MATCH_TYPE::_TRUE){
					g[v0].options.erase(g[v0].options.begin());
					v1=match.second; //frontier
						edge.first= boost::add_edge(v0, v1, g).first; //assumes edge added
						edge.second=true; //just means that the edge is valid
						g[edge.first]=sk.second;//t.direction;
				}
				else{
					edge= add_vertex_now(v0, v1,g,sk.first.Di, sk.second); //addVertex
					g[edge.first.m_target].label=sk.first.label; //new edge, valid
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, errorMap, iteration);
					gt::adjustProbability(g, edge.first);
				}
				applyTransitionMatrix(g, v1, t.direction, er.ended, v0);
				g[v1].phi=evaluationFunction(er);
				std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toPrune =(propagateD(v1, v0, g,&propagated, &closed)); //og v1 v0
				v0_exp=v0;
				options=g[v0_exp].options;
				v0=v1;			
				pruneEdges(toPrune,g, v, v0_exp, priorityQueue, toRemove);
			
			}while(t.direction !=DEFAULT & int(g[v0].options.size())!=0);
		evaluationQueue.push_back(v1);
		}
	}
	backtrack(evaluationQueue, priorityQueue, closed, g);
	bestNext=priorityQueue[0];
	direction = g[boost::in_edges(bestNext, g).first.dereference()].direction;
}while(g[bestNext].options.size()>0);
return toRemove;
}

std::vector <vertexDescriptor> Configurator::splitTask( vertexDescriptor v, TransitionSystem& g, Direction d, vertexDescriptor src){
	std::vector <vertexDescriptor> split={v};

	if (d ==RIGHT || d==LEFT){
		return split;
	}
	if (g[v].outcome != simResult::crashed){
		return split;
	}
	//if (src!=TransitionSystem::null_vertex()){
	if (auto ie=gt::inEdges(g, src, DEFAULT), stop_edges=gt::inEdges(g, src, STOP); !ie.empty()|| !stop_edges.empty()){
		split.insert(split.begin(), src);
		g[src].outcome=simResult::safeForNow;
	}
	//}
	auto first_edge=boost::edge(src, v, g);
	vertexDescriptor v1=v;
	float nNodes = g[v].distance()/simulationStep, og_phi=g[v].phi;
	b2Transform endPose = g[v].endPose;
	Task::Action a;
	a.init(d);
	while(nNodes>1){
		if(nNodes >1){
			b2Vec2 step_v(simulationStep*endPose.q.c, simulationStep*endPose.q.s);
			g[v].options = {d};
			g[v].endPose = g[v].start+b2Transform(step_v, b2Rot(0));
			g[first_edge.first].step= gt::distanceToSimStep(g[v].distance(), a.getLinearSpeed());
			first_edge=add_vertex_retro(v, v1,g, g[v].Dn); //passing on the disturbance
			g[v1].Di=g[v].Di;
			g[v1].start=g[v].endPose;
			g[v].phi=NAIVE_PHI;
			g[first_edge.first].direction=d;
			g[v].outcome=simResult::safeForNow;
			split.push_back(v1);
			nNodes--;
		}
		if (nNodes<=1){
			g[v1].endPose = endPose;
			g[first_edge.first].step= gt::distanceToSimStep(g[v1].distance(), a.getLinearSpeed());	
			g[v1].outcome=simResult::crashed;
			g[v1].phi=og_phi;
		}

		v=v1;

	}
	return split;
}


void Configurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, const std::set<vertexDescriptor>& closed, TransitionSystem&g){
	for (vertexDescriptor v:evaluation_q){
		std::vector <edgeDescriptor> ie=gt::inEdges(g, v);
		std::pair<bool, edgeDescriptor> ep= gt::visitedEdge(ie, g,currentVertex);
		if (!ep.first){
			ep=gt::getMostLikely(g, ie, iteration);
		}
		b2Transform start=g[ep.second.m_source].endPose;
		vertexDescriptor src=ep.second.m_source;
		std::vector <vertexDescriptor> split = splitTask(v, g, g[ep.second].direction, src);
		for (vertexDescriptor split_v:split){
				EndedResult local_er=estimateCost(g[split_v],start, g[ep.second].direction);
				g[split_v].phi=evaluationFunction(local_er);
				applyTransitionMatrix(g, split_v, g[ep.second].direction, local_er.ended,src);
				addToPriorityQueue(split_v, priority_q, g, closed);
				src=split_v;
		}
	}
	evaluation_q.clear();
}

std::vector<std::pair<vertexDescriptor, vertexDescriptor>> Configurator::propagateD(vertexDescriptor v1, vertexDescriptor v0,TransitionSystem&g, std::vector<vertexDescriptor>*propagated, std::set <vertexDescriptor>*closed){
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> deletion;
	if (g[v1].outcome == simResult::successful ){
		return deletion;
	}
	vertexDescriptor p=TransitionSystem::null_vertex();
	std::pair <edgeDescriptor, bool> ep= boost::edge(v0, v1, g);
	Disturbance dist = g[v1].Dn;
	while (ep.second){
		if(g[ep.first].direction!=DEFAULT ){
			if (g[ep.first].direction==STOP){
				g[ep.first.m_target].Dn = dist;
			}
			break;
		}
		if (ep.first.m_target!=v1){
			g[ep.first.m_target].Dn = dist;
			EndedResult er=estimateCost(g[ep.first.m_target], g[ep.first.m_source].endPose,g[ep.first].direction); //reassign cost
			g[ep.first.m_target].phi =evaluationFunction(er);	
			std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> match= findMatch(ep.first.m_target, g, g[ep.first].direction);
			if ( match.first){
				std::pair<vertexDescriptor, vertexDescriptor>pair(ep.first.m_target, match.second);
				deletion.push_back(pair);			//first is eliminated, the second is its match
				p=(pair.second);
	
			}
			else{
				p=(ep.first.m_target);
			}
			if (auto found=closed->find(p); found!=closed->end()){
				closed->erase(found);
			}
			propagated->push_back(p);
		}
		ep.second= boost::in_degree(ep.first.m_source, g)>0;
		if (!ep.second){
			return deletion;
		}
		ep.first= *(boost::in_edges(ep.first.m_source, g).first);
	}	
	return deletion;
}

void Configurator::pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> vertices, TransitionSystem& g, vertexDescriptor& src, vertexDescriptor& active_src, std::vector <vertexDescriptor>& pq, std::vector<std::pair<vertexDescriptor, vertexDescriptor>>&toRemove){ //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end
	for (std::pair<vertexDescriptor, vertexDescriptor> pair:vertices){
		if (pair.first==src){
			src=pair.second;
		}
		if (pair.first==active_src){
			active_src=pair.second;
		}
		std::vector<edgeDescriptor> ie =gt::inEdges(g, pair.second, DEFAULT); //first vertex that satisfies that edge requirement
		std::vector <edgeDescriptor> toReassign=gt::inEdges(g, pair.first, DEFAULT);
		edgeDescriptor e =edgeDescriptor(), r_visited=edgeDescriptor(), e2;
		std::pair <bool,edgeDescriptor> ep2 = gt::visitedEdge(toReassign, g);
		e2=ep2.second;
		if (ie.empty()){
		}
		else{
			e=ie[0];
			toReassign.push_back(e);
		}
		gt::update(e, std::pair <State, Edge>(g[pair.first], g[e2]),g, pair.second==currentVertex, errorMap, iteration);
		float match_distance=10000;
		for (edgeDescriptor r:toReassign){ //reassigning edges
			auto new_edge= gt::add_edge(r.m_source, pair.second, g, iteration);
		}
		boost::clear_vertex(pair.first, g);
		toRemove.push_back(pair);
		for (int i=0; i<pq.size(); i++){ //REMOVE FROM PQ
			if(pq[i]==pair.first){
				pq.erase(pq.begin()+i);
			}
		}
		gt::adjustProbability(g, e);
	}
}

void Configurator::clearFromMap(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> matches , TransitionSystem&g, std::unordered_map<State*, ExecutionError>map){
		for (std::pair<vertexDescriptor, vertexDescriptor> pair:matches){
			if (auto it=map.find(g[pair.first].ID); it!=map.end()){
				ExecutionError exer = map.at(transitionSystem[pair.first].ID);
				map.insert_or_assign(transitionSystem[pair.second].ID, exer);
				map.erase(it);
				break;
			}

		}
}

std::vector <vertexDescriptor> Configurator::planner( TransitionSystem& g, vertexDescriptor src, vertexDescriptor goal, bool been, const Task* custom_ctrl_goal, bool *finished){
	std::vector <vertexDescriptor> plan;
	std::vector<std::vector<vertexDescriptor>> paths;
	paths.push_back(std::vector<vertexDescriptor>()={src});
	std::vector <Frontier> frontier_v;
	bool run=true, _finished=false;
	std::vector <Frontier> priorityQueue={Frontier(src, std::vector<vertexDescriptor>())};
	Task overarching_goal;
	if (NULL==custom_ctrl_goal){
		overarching_goal=controlGoal;
	}
	else{
		overarching_goal=*custom_ctrl_goal;
	}
	int no_out=0;
	std::vector <vertexDescriptor> add;
	std::vector<std::vector<vertexDescriptor>>::reverse_iterator path= paths.rbegin();
	vertexDescriptor path_end=src;
	do{
		frontier_v=frontierVertices(src, g, DEFAULT, been);
		if (src==currentVertex){
		}
		priorityQueue.erase(priorityQueue.begin());
		for (Frontier f: frontier_v){ //add to priority queue
			planPriority(g, f.first);
			addToPriorityQueue(f, priorityQueue, g);
		}
		if (priorityQueue.empty()){
			break;
		}
		src=priorityQueue.begin()->first;
		add=std::vector <vertexDescriptor>(priorityQueue.begin()->second.begin(), priorityQueue.begin()->second.end());
		add.push_back(src);
		std::pair<edgeDescriptor, bool> edge(edgeDescriptor(), false);
		std::vector<vertexDescriptor>::reverse_iterator pend=(path->rbegin());
		while (!edge.second){//|| ((*(pend.base()-1)!=goal &goal!=TransitionSystem::null_vertex())&!controlGoal.checkEnded(g[*(pend.base()-1)]).ended)
			vertexDescriptor end=*(pend.base()-1);
			edge= boost::edge(end,add[0], g);
			if (!add.empty()&!edge.second & path!=paths.rend()){ //if this path does not have an edge and there are 
													//other possible paths, go to previous paths
				if (pend.base()-1!=(path->begin())){ //if the current vertex is not the root of the path
					pend++;
				}
				else{
					path++; //go back a previously explored path
					pend=(*path).rbegin(); 
				}
			}
			else if (edge.second & pend.base()!=path->rbegin().base()){  //if there is an edge with the end of current path
				bool found=0;
				for (auto _p=paths.rbegin(); _p!=paths.rend(); _p++ ){
					if (std::vector <vertexDescriptor>(path->begin(), pend.base())==*_p){
						path=_p;
						found=1;
					}
				}
				if (!found){
				paths.emplace_back(std::vector <vertexDescriptor>(path->begin(), pend.base()));
				path=paths.rbegin();				
				}
				break;
			}
			if ( path==paths.rend()) { //if there are no other paths
				paths.push_back(std::vector<vertexDescriptor>()); //make a new one
				path=paths.rbegin();
				break;
			}
		}
		for (vertexDescriptor c:add){
			g[c].label=VERTEX_LABEL::UNLABELED;
			path->push_back(c);	
			path_end=c;			
		}
		_finished=overarching_goal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended;
		if (NULL!=finished){
			*finished=_finished;
		}
	}while(!priorityQueue.empty() && (path_end!=goal && !(_finished)));
	auto vs=boost::vertices(g);
	float final_phi=10000;
	for (std::vector<vertexDescriptor> p: paths){
		vertexDescriptor end_plan= *(p.rbegin().base()-1);
		if (end_plan==goal){
			plan=std::vector(p.begin()+1, p.end());
			break;
		}
		else if (g[end_plan].phi<final_phi){
			plan=std::vector(p.begin()+1, p.end());
			final_phi=g[end_plan].phi;
		}
	}
	return plan;

}






EndedResult Configurator::estimateCost(State &state, b2Transform start, Direction d){
	EndedResult er = controlGoal.checkEnded(state);
	Task t(state.Dn, d, start);
	er.cost += t.checkEnded(state.endPose).estimatedCost;
	if (state.outcome==simResult::crashed){
		er.cost+=2;
	}
	return er;
}


float Configurator::evaluationFunction(EndedResult er){ 
	return (abs(er.estimatedCost)+abs(er.cost))/2; //normalised to 1
}



void Configurator::printPlan(std::vector <vertexDescriptor>* p){
	if (p==NULL){
		printf("currently executed plan:\t");
		p=&planVertices;
	}
	else{
		printf("provisional plan:\t");
	}
	std::vector <vertexDescriptor> plan= *p;
	vertexDescriptor pre=currentVertex;
	for (vertexDescriptor v: plan){
		std::pair <edgeDescriptor, bool> edge=boost::edge(pre, v, transitionSystem);
		if (!edge.second){
			printf("no edge: %i-> %i", edge.first.m_source, edge.first.m_target);
		}
		else{
			auto a=dirmap.find(transitionSystem[edge.first].direction);
			printf("%i, %s, ", edge.first.m_target, (*a).second);
		}
		pre=edge.first.m_target;
		}
	printf("\n");
}



void Configurator::start(){
	if (ci == NULL){
		throw std::invalid_argument("no data interface found");
		return;
	}
	running =1;
	if (t!=NULL){ //already running
		return;
	}
	t= new std::thread(Configurator::run, this);

}

void Configurator::stop(){
	running =0;
	if (t!=NULL){
		t->join();
		delete t;
		t=NULL;
	}
}

void Configurator::registerInterface(ConfiguratorInterface * _ci){
	ci = _ci;
}

void Configurator::run(Configurator * c){
	while (c->running){
		if (c->ci->stop){
			c->ci=NULL;
		}
		if (c->ci == NULL){
			printf("null pointer to interface\n");
			c->running=0;
			return;
		}
		if (c == NULL){
			printf("null pointer to configurator\n");
			c->running=0;
			return;
		}
		if (c->ci->isReady()){
			c->ci->ready=0;
			c->data2fp= CoordinateContainer(c->ci->data2fp);
			c->Spawner();
		}
	}

}

void Configurator::unexplored_transitions(TransitionSystem& g, const vertexDescriptor& v){
	std::vector <Direction> to_remove;
	for (int i=0; i<g[v].options.size(); i++){
		for (edgeDescriptor &e: gt::outEdges(g, v, g[v].options[i])){
			if (g[e.m_target].visited()){
				to_remove.push_back(g[e].direction);
			}
		}
	}
	for (Direction & d:to_remove){
		auto it=std::find(g[v].options.begin(), g[v].options.end(), d);
		if (it !=g[v].options.end()){
			g[v].options.erase(it);
		}
	}
}

void Configurator::transitionMatrix(State& state, Direction d, vertexDescriptor src){
	Task temp(controlGoal.disturbance, DEFAULT, state.endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	if (state.outcome == simResult::safeForNow){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
			if (state.nodesInSameSpot<maxNodesOnSpot){
				if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
					state.options.push_back(temp.direction);
					state.options.push_back(getOppositeDirection(temp.direction).second);
				}
				else{
					int random= rand();
					if (random%2==0){
						state.options = {LEFT, RIGHT};
					}
					else{
						state.options = {RIGHT, LEFT};
					}
				}
			}
			}
	}
	else if (state.outcome==simResult::successful) { //will only enter if successful
		if (d== LEFT || d == RIGHT){
			state.options = {DEFAULT};
			if (src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.disturbance.pose().p)<0){
				state.options.push_back(d);
			}
		}
		else {
			if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
				state.options.push_back(temp.direction);
				state.options.push_back(getOppositeDirection(temp.direction).second);
				state.options.push_back(DEFAULT);
			}
			else{
				state.options={DEFAULT};
			}

		}

	}
}

void Configurator::applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src){
	if (!g[v0].options.empty()){
		return;
	}
	if (controlGoal.endCriteria.hasEnd()){
		if (ended){
			return;
		}
	}
	else if(round(g[v0].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		if (controlGoal.getAffIndex()==PURSUE){
		}
		return;
	}
	transitionMatrix(g[v0], d, src);
	unexplored_transitions(g, v0);
}


void Configurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, TransitionSystem &g, const std::set <vertexDescriptor>& closed){
	if (g[v].outcome==simResult::crashed){
		return;
	}
	for (auto i =queue.begin(); i!=queue.end(); i++){
		bool expanded=0;
		auto found=closed.find(v); 
		if (g[v].phi <abs(g[*i].phi) && found==closed.end()){
			queue.insert(i, v);
			return;
		}
	}
	queue.push_back(v);
}


void Configurator::addToPriorityQueue(Frontier f, std::vector<Frontier>& queue, TransitionSystem &g, vertexDescriptor goal){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (g[f.first].phi <abs(g[(*i).first].phi)){
			queue.insert(i, f);
			return;
		}
	}
	queue.push_back(f);
}
std::pair <edgeDescriptor, bool> Configurator::maxProbability(std::vector<edgeDescriptor> ev, TransitionSystem& g){
	std::pair <edgeDescriptor, bool> result;
	if (ev.empty()){
		result.second=false;
		return result;
	}
	result.first = ev[0];
	result.second=true;
	for (edgeDescriptor e :ev){
		if (g[e].probability>g[result.first].probability){
			result.first =e;
		}
	}
	return result;
}



void Configurator::adjustStepDistance(vertexDescriptor v, TransitionSystem &g, Task * t, float& step, std::pair<bool,vertexDescriptor> tgt){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);
	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
	float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
	float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
	if (currentTask.getAction().getOmega()!=0){
		float remainingAngle = currentTask.endCriteria.angle.get()-abs(theta_obs);
		if (t->direction==getOppositeDirection(currentTask.direction).second){
			remainingAngle=M_PI-remainingAngle;
		}
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(currentTask.getAction().getLinearSpeed()>0){
		step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
	}			// -estimated distance covered
}


std::vector <edgeDescriptor> Configurator::inEdgesRecursive(vertexDescriptor v, TransitionSystem& g, Direction d){
	 	std::vector <edgeDescriptor> result;
		edgeDescriptor e; 
		do{
			std::vector <edgeDescriptor> directionEdges=gt::inEdges(g, v, d);
			if (directionEdges.empty()){
				break;
			}
			auto eit=std::max_element(directionEdges.begin(), directionEdges.end()); //choosing the most recent edge
			e = *eit;
			result.push_back(e);
			v=e.m_source;
		}while (g[e].direction==d);
	return result;
} 

std::vector <Frontier> Configurator::frontierVertices(vertexDescriptor v, TransitionSystem& g, Direction d, bool been){
	std::vector <Frontier> result;
	std::pair<edgeDescriptor, bool> ep=boost::edge(movingVertex, v, g); 
	vertexDescriptor v0=v, v1=v, v0_exp;
	//do{
		if ((controlGoal.disturbance.getPosition()-g[v].endPose.p).Length() < DISTANCE_ERROR_TOLERANCE){
		}
		else{
			auto es=boost::out_edges(v, g);
			for (auto ei=es.first; ei!=es.second; ei++){
			std::vector <vertexDescriptor>connecting;
			auto ei2=ei, ei3=ei;
			auto es2=boost::out_edges((*ei).m_target, g);
			auto es3=es2;
			std::vector <vertexDescriptor>connecting2;
			do {
				//=connecting;
				if (g[(*ei3).m_target].visited() || been){
					if (!g[(*ei3).m_target].visited()){
						EndedResult er = estimateCost(g[(*ei3).m_target], g[(*ei3).m_source].endPose, g[*ei3].direction);
						g[(*ei3).m_target].phi=evaluationFunction(er);
					}
					if (g[(*ei3)].direction==d){
						Frontier f;
						//f_added=1;
						f.first= (*ei3).m_target;
						f.second=connecting2;
						result.push_back(f);
						if (ei3!=ei){
							ei3++;
							//connecting2.clear();
							//ei2=ei3;
						}
						else{
							connecting.clear();
							break;
						}
					}
					else if (ei3==ei){
						connecting.push_back((*ei3).m_target);
						connecting2=connecting;
						es3=boost::out_edges((*ei3).m_target,g);
						ei3=es3.first;
						ei2=ei3;
						es2=es3;
					}
					else if (ei2!=ei){
						connecting2.push_back((*ei3).m_target);
						es3=boost::out_edges((*ei3).m_target,g);
						ei3=es3.first;
						ei2++;
					}
				}
				else if (ei3!=ei){
					ei3++;
				}
				if(ei3==es3.second){
					if (ei3!=ei2 & ei2!=ei){
						ei2++;
						ei3=ei2;
						es3=es2;
					}					
				}
			}while (ei3!=es3.second);
	}
	}

	return result;
}





std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(State s, TransitionSystem& g, State * src, Direction dir, StateMatcher::MATCH_TYPE match_type, std::vector <vertexDescriptor> * others, bool relax){
	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex());
	auto vs= boost::vertices(g);
	float prob=0, sum=10000;
	//need to find best match too
	ComparePair comparePair;
	std::set <std::pair <vertexDescriptor, float>, ComparePair>others_set(comparePair);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=true;
		std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
		Tmatch=!ie.empty()||dir==Direction::UNDEFINED;
		StateDifference sd(s, g[v]);
		bool condition=0;
		StateMatcher::MATCH_TYPE m;
		float sum_tmp=sd.get_sum(match_type);
		if (!relax){
			m= matcher.isMatch(s, g[v], src, &sd);
			condition=matcher.match_equal(m, match_type);
		}
		else{
			condition= sum_tmp<sum;
		}
		if ( condition&& v!=movingVertex && boost::in_degree(v, g)>0 &&Tmatch ){ 
			sum=sum_tmp;
			if (NULL!=others){
			 	others_set.emplace(std::pair< vertexDescriptor, float>(v, sum));
			 }
				if (!relax){
					result.first= m;
				}
				else{
					result.first=match_type;
				}
				result.second=v;
		}
	}
	if (others==NULL){
		return result;
	}
	for (auto vp:others_set){
		others->push_back(vp.first);
	}
	return result;
}

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(vertexDescriptor v, TransitionSystem& g, Direction dir, StateMatcher::MATCH_TYPE match_type, std::vector <vertexDescriptor>* others){
	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::_FALSE, TransitionSystem::null_vertex());
	auto vs= boost::vertices(g);
	int nObs=0;
	for (auto vi=vs.first; vi!= vs.second; vi++){
		if (*vi!=v){
			std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
			bool Tmatch=true;
			Tmatch= !ie.empty()||dir==Direction::UNDEFINED;
			if (StateMatcher::MATCH_TYPE m=matcher.isMatch(g[v], g[*vi]); matcher.match_equal(m, match_type) && *vi!=movingVertex &Tmatch & boost::in_degree(*vi, g)>=ie.size()){ //
				if(g[v].nObs>nObs){
				result.first=matcher.isMatch(g[v], g[*vi]);
				result.second=*vi;
				nObs=g[v].nObs;
				if (NULL!=others){
					others->push_back(result.first);
				}
			}
		}
		}
	}
	return result;
}


void Configurator::changeStart(b2Transform& start, vertexDescriptor v, TransitionSystem& g){
	if (g[v].outcome == simResult::crashed && boost::in_degree(v, g)>0){
		edgeDescriptor e = boost::in_edges(v, g).first.dereference();
		start = g[e.m_source].endPose;
	}
	else{
		start=g[v].endPose;
	}
}


ExecutionError Configurator::trackTaskExecution(Task & t){
	ExecutionError error;
	t.motorStep--;		
	updateGraph(transitionSystem, error);//lateral error is hopefully noise and is ignored
	if(t.motorStep==0){
		t.change=1;
	}
	return error;
}

b2Transform Configurator::assignDeltaPose(Task::Action a, float timeElapsed){
	b2Transform result;
	float theta = a.getOmega()* timeElapsed;
	result.p ={a.getLinearSpeed()*cos(theta),a.getLinearSpeed()*sin(theta)};
	result.q.Set(a.getOmega());
	return result;
}


int Configurator::motorStep(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		}
		else if (a.getLinearSpeed()>0){
			result = (simulationStep)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
    }

std::vector <vertexDescriptor> Configurator::changeTask(bool b, int &ogStep, std::vector <vertexDescriptor> pv){
	if (!b){
		return pv;
	}
	if (planning){
		if (pv.empty()){
				currentTask=Task(STOP);
			return pv;
		}
		std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, pv[0], transitionSystem);
		currentVertex= pv[0];
		pv.erase(pv.begin());
		currentEdge=ep.first;
		boost::clear_vertex(movingVertex, transitionSystem);
		movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingEdge].direction=transitionSystem[ep.first].direction;
		transitionSystem[movingEdge].step=currentTask.motorStep;
		currentTask = Task(transitionSystem[currentEdge.m_source].Dn, transitionSystem[currentEdge].direction, b2Transform(b2Vec2(0,0), b2Rot(0)), true);
		currentTask.motorStep = transitionSystem[currentEdge].step;
	}
	else{
		if (transitionSystem[currentVertex].Dn.isValid()){
			currentTask = Task(transitionSystem[currentVertex].Dn, DEFAULT); //reactive
		}
		else if(currentTask.direction!=DEFAULT){
				currentTask = Task(transitionSystem[currentVertex].Dn, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}
		gt::fill(simResult(), &transitionSystem[currentVertex]);
		currentTask.motorStep = motorStep(currentTask.getAction());
		transitionSystem[movingEdge].step=currentTask.motorStep;
	}
	ogStep = currentTask.motorStep;
	return pv;
}

void Configurator::trackDisturbance(b2Transform & pose, Task::Action a, float error){
	float angleTurned =MOTOR_CALLBACK*a.getOmega();
	pose.q.Set(pose.q.GetAngle()-angleTurned);	
	float distanceTraversed = 0;
	float initialL = pose.p.Length();
	if(fabs(error)<TRACKING_ERROR_TOLERANCE){
		distanceTraversed= MOTOR_CALLBACK*a.getLinearSpeed();
	}
	else{
		distanceTraversed=error;
	}
	pose.p.x=cos(pose.q.GetAngle())*initialL-cos(angleTurned)*distanceTraversed;
	pose.p.y = sin(pose.q.GetAngle())*initialL-sin(angleTurned)*distanceTraversed;
}

void Configurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:planVertices){
		if (p==v){
       		g[v].phi-=.1;
			break;
		}
    } 
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, Task& task){
	math::applyAffineTrans(deltaPose, task.start);
	applyAffineTrans(deltaPose, task.disturbance);
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, TransitionSystem& g){
	auto vPair =boost::vertices(g);
	for (auto vIt= vPair.first; vIt!=vPair.second; ++vIt){ //each node is adjusted in explorer, so now we update
	if (*vIt!=movingVertex){
		math::applyAffineTrans(deltaPose, g[*vIt]);
	}
}
}

void Configurator::applyAffineTrans(const b2Transform& deltaPose, Disturbance& d){
	if (d.getAffIndex()!=NONE){
		math::applyAffineTrans(deltaPose, d.bf.pose);
	}
}


void Configurator::updateGraph(TransitionSystem&g, ExecutionError error){
	b2Transform deltaPose;
	float angularDisplacement= getTask()->getAction().getOmega()*MOTOR_CALLBACK +error.theta();
	float xdistance=cos(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	float ydistance=sin(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
	deltaPose=b2Transform(b2Vec2(xdistance,
					ydistance), 
					b2Rot(angularDisplacement));
	applyAffineTrans(deltaPose, g);
	applyAffineTrans(deltaPose, controlGoal);
}

