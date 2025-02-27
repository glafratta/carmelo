#include "configurator.h"
#include <chrono>


void math::applyAffineTrans(const b2Transform& deltaPose, Task* task){
	math::applyAffineTrans(-deltaPose, task->start);
	applyAffineTrans(-deltaPose, task->disturbance);
}


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
	printf("dummy, current edge = %i, %i\n", src, currentVertex);
	transitionSystem[movingEdge].direction=STOP;
	transitionSystem[currentEdge].direction=STOP;
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
	iteration++; //iteration set in getVelocity
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
	if (planning){
		vertexDescriptor src=movingVertex; 
		if (transitionSystem.m_vertices.size()==1){
			dummy_vertex(currentVertex);
			currentTask.change=1;
		}
		if (!planVertices.empty() || currentTask.motorStep!=0){ //
			src=movingVertex;
		}
		else{
			src=currentVertex;
		}
		resetPhi(transitionSystem);
		planVertices=explorer(src, transitionSystem, currentTask, world);
		if (debugOn){
			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, planVertices, currentVertex);
		}		
		ts_cleanup(&transitionSystem);
		if (planVertices.empty() && (!transitionSystem[currentVertex].visited() || currentTask.motorStep==0)){ //currentv not visited means that it wasn't observed ()
			printf("no plan, searchign from %i\n", src);
			planVertices= planner(transitionSystem, currentVertex); //src
		}
		else{
			printf("recycled plan in explorer:\n");
		}
		printPlan(&planVertices);
//		boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);


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
		//adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
		worldBuilder.buildWorld(world, data2fp, transitionSystem[movingVertex].start, currentTask.direction); //was g[v].endPose
		simResult result = simulate(currentTask, world, _simulationStep); //transitionSystem[currentVertex],transitionSystem[currentVertex],
		gt::fill(result, transitionSystem[currentVertex].ID, &transitionSystem[currentEdge]);
		currentTask.change = transitionSystem[currentVertex].outcome!=simResult::successful;
		if (currentTask.change){
			printf("crashed, curre\n");
		}
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
	//printf("end explor\n");
	return 1;
}

void Configurator::resetPhi(TransitionSystem&g){
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		g[*vi].resetVisited();
		g[*vi].options.clear();
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
Disturbance Configurator::getDisturbance(TransitionSystem&g, const  vertexDescriptor& v, b2World & world, const Direction& dir, const b2Transform& start){
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=gt::inEdges(g, v, UNDEFINED);
		std::vector <edgeDescriptor> out=gt::outEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);
			if (visited.first ||out.empty()){
				if (g[v].Di.isValid() && g[v].Di.affordanceIndex==AVOID && g[visited.second].direction!=dir){
					Task task(g[v].Di, DEFAULT, g[v].endPose, true);
					Robot robot(&world);
					robot.body->SetTransform(task.start.p, task.start.q.GetAngle());
					b2AABB box =worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
					b2Fixture *sensor =GetSensor(robot.body);
					bool overlap=overlaps(robot.body, &g[v].Di) && sensor;
					worldBuilder.world_cleanup(&world);
					if (overlap){
						Disturbance Di= g[v].Di;
						Di.bf.pose+= start-g[v].endPose;
						return Di;
					}
				}
				//check if Di was eliminated 
				return controlGoal.disturbance;
			}
			else if (v==movingVertex){
				return g[v].Di;
			}
	}
	Disturbance Dn= g[v].Dn;
	Dn.bf.pose+= start-g[v].endPose;
	return Dn;
	//return controlGoal.disturbance;
}


Task Configurator::task_to_execute(const TransitionSystem & g, const edgeDescriptor& e){
	Task t=controlGoal;
	if (Disturbance Dn= g[e.m_target].Dn; Dn.getAffIndex()==AVOID){
		Disturbance Di= Dn;
		Di.affordanceIndex=PURSUE;
		t=Task(Di, g[e].direction, b2Transform_zero, true);
		float distance = g[e.m_target].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance));
	}
	else{
		t=Task(g[e.m_target].Di, g[e].direction, b2Transform_zero, true);

	}
	return t;

}


simResult Configurator::simulate(Task  t, b2World & w, float _simulationStep){ //State& state, State src, 
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
	result =t.bumping_that(w, iteration, robot.body, debugOn, remaining, _simulationStep); //default start from 0
	worldBuilder.world_cleanup(&w);
	//approximate angle to avoid stupid rounding errors
	float approximated_angle=approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode);
	result.endPose.q.Set(approximated_angle);
	return result;
	}


std::vector<vertexDescriptor> Configurator::explorer(vertexDescriptor v, TransitionSystem& g, Task t, b2World & w){
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	Direction direction=currentTask.direction;
	std::vector <vertexDescriptor> priorityQueue = {v}, evaluationQueue, plan_prov=planVertices;
	std::set <vertexDescriptor> closed;
	b2Transform start= b2Transform_zero, shift=b2Transform_zero, shift_start=shift;
	EndedResult er;
	printf("v=%i, initial plan size=%i\n",v, plan_prov.size());
	printf("GOAL IS: ");
	debug::print_pose(controlGoal.disturbance.pose());
	do{
		v=bestNext;
		closed.emplace(*priorityQueue.begin().base());
		priorityQueue.erase(priorityQueue.begin());
		er = controlGoal.checkEnded(g[v], t.direction);
		applyTransitionMatrix(g, v, direction, er.ended, v, plan_prov);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0_exp=v;
			std::vector <Direction> options=g[v0_exp].options;
			while (!options.empty()){
				options.erase(options.begin());
				v0=v0_exp; //node being expanded
				v1 =v0; //frontier
				std::vector <vertexDescriptor> propagated;
				do {
				start=g[v0].endPose +shift;
				Disturbance Di=getDisturbance(g, v0, w, g[v0].options[0], start);
				t = Task(Di, g[v0].options[0], start, true);//need to update end crit
				std::pair <State, Edge> sk(State(start, Di), Edge(g[v0].options[0]));
				//float _simulationStep=BOX2DRANGE;
				//adjustStepDistance(v0, g, &t, _simulationStep);
				adjust_simulated_task(v0, g, &t);
				worldBuilder.buildWorld(w, data2fp, t.start, t.direction, t.disturbance, 0.15, WorldBuilder::PARTITION); //was g[v].endPose
				printf("v0=%i, dir=%s\n", v0, (*dirmap.find(t.direction)).second);
				simResult sim=simulate(t, w, _simulationStep); //sk.first, g[v0], 
				if (v==0 && sim.resultCode==sim.crashed){
					printf("IM GONNA CRASH!!!! at");
					debug::print_pose(sim.collision.pose());
				}
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.direction=t.direction;
				sk.second.it_observed=iteration;
				er  = estimateCost(sk.first, g[v0].endPose, sk.second.direction);
				State * source=NULL;
				StateMatcher::MATCH_TYPE vm= matcher.isMatch(g[v], g[currentEdge.m_source]); //see if we are at the beginning of the exploration:
																					//v=0 and currentEdge =src will match so we try to prevent
																			//changing the movign vertex which is by default the origin
				bool closest_match=false, match_task=true;
				StateMatcher::MATCH_TYPE desired_match=StateMatcher::MATCH_TYPE::ABSTRACT;
				std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(sk.first, g, g[v0].ID, t.direction, desired_match, NULL, closest_match, match_task );		//, closest_match	
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher.match_equal(match.first, desired_match)){
					g[v0].options.erase(g[v0].options.begin());
					v1=match.second; //frontier
					printf("match with %i\n", v1);
						edge= gt::add_edge(v0, v1, g, iteration, t.direction); //assumes edge added
						if (edge.second){
							//printf("added edge: %i -> %i, step=%i\n", v0, v1, sk.second.step);
							g[edge.first]=sk.second; //doesn't update motorstep
						}
					if (currentTask.motorStep==0){
						std::vector <vertexDescriptor> task_vertices=gt::task_vertices(v1, g, iteration, currentVertex);
						vertexDescriptor task_start= task_vertices[0];
						if (plan_prov.empty()){
							bool finished=false, been=matcher.match_equal(match.first, StateMatcher::ABSTRACT); //(match.first==StateMatcher::DISTURBANCE); //ADD representation of task but shifted
							//shift here?
							Task controlGoal_adjusted= controlGoal;
							shift_start= b2MulT(b2MulT(sk.first.start, controlGoal.start), g[task_start].start);
							math::applyAffineTrans(shift_start, &controlGoal_adjusted); //as start
							auto plan_tmp=planner(g, task_start, TransitionSystem::null_vertex(), been, &controlGoal_adjusted, &finished);
							bool filler=0;
							// if (match.second!=v){
							// 	shift= b2MulT(g[task_start].start, start);
							// }
							if (finished){
								plan_prov=plan_tmp;
								if (planVertices.empty()){ // task_start==currentVertex instead of pv empty
									printf("inserting current vertex\n");
									plan_prov.insert(plan_prov.begin(), task_start);
								}
								boost::remove_edge(edge.first, g);
								edge= gt::add_edge(v0, task_start, g, iteration, g[edge.first].direction);
								printf("edge %i -> %i added\n", v0, task_start);
								if (t.direction== g[edge.first].direction){
									g[v0].options.clear();
								}
								else{
									g[v0].options={g[edge.first].direction};
								}
							}
						}
						if (planVertices.empty()){
							shift_states(g, task_vertices, shift_start);
						}
					}
					
				}
				else{
					auto out_expected=gt::outEdges(g, v0, t.direction);
					edge= add_vertex_now(v0, v1,g,sk.first.Di, sk.second); //addVertex
					g[edge.first.m_target].label=sk.first.label; //new edge, valid
					if (!out_expected.empty()){
						vertexDescriptor exp=out_expected[0].m_target;
						printf("thought it'd be vertex %i , end pose:", exp );
						debug::print_pose(g[exp].endPose);

					}
					auto d_print=dirmap.find(t.direction);
					printf("added v %i to %i, direction %s", v1, v0, (*d_print).second);
					shift=b2Transform_zero;
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, iteration);
					gt::adjustProbability(g, edge.first); //new_edge to allow to adjust prob if the sim state has been previously ecountered and split
				}
				applyTransitionMatrix(g, v1, t.direction, er.ended, v0, plan_prov);
				g[v1].phi=evaluationFunction(er);
				propagateD(v1, v0, g,&propagated, &closed); //og v1 v0
				v0_exp=v0;
				options=g[v0_exp].options;
				v0=v1;						
			}while(t.direction !=DEFAULT & int(g[v0].options.size())!=0);
		evaluationQueue.push_back(v1);
		}
	}
	backtrack(evaluationQueue, priorityQueue, closed, g, plan_prov);
	bestNext=priorityQueue[0];
	printf("best=%i end", bestNext);
	debug::print_pose(g[bestNext].endPose);
	std::vector <edgeDescriptor> best_in_edges= gt::inEdges(g,bestNext);
	if (best_in_edges.empty()){
		direction=currentTask.direction;
	}
	else{
		direction = g[best_in_edges[0]].direction;
		g[best_in_edges[0]].it_observed=iteration;
	}
}while(g[bestNext].options.size()>0 && !er.ended);
printf("finished exploring, plan =%i\n", plan_prov.size());
return plan_prov;
}

std::vector <vertexDescriptor> Configurator::splitTask( vertexDescriptor v, TransitionSystem& g, Direction d, vertexDescriptor src){
	std::vector <vertexDescriptor> split={v};
	auto first_edge=boost::edge(src, v, g); //assumes exists

	if (gt::check_edge_direction(first_edge, g, RIGHT)|| gt::check_edge_direction(first_edge, g, LEFT)){ //d
		return split;
	}
	if (g[v].outcome != simResult::crashed){
		return split;
	}
	if (auto ie=gt::inEdges(g, src, DEFAULT), stop_edges=gt::inEdges(g, src, STOP); !ie.empty()|| !stop_edges.empty()){
		split.insert(split.begin(), src);
		g[src].outcome=simResult::safeForNow;
	}
	vertexDescriptor v1=v;
	float nNodes = g[v].distance()/simulationStep, og_phi=g[v].phi;
	b2Transform endPose = g[v].endPose;
	Task::Action a;
	a.init(d);
	while(nNodes>1){
		State s_tmp=State(g[v]);
		if(nNodes >1){
			b2Vec2 step_v(simulationStep*endPose.q.c, simulationStep*endPose.q.s);
			s_tmp.endPose=g[v].start+b2Transform(step_v, b2Rot(0));
			std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(s_tmp, g, NULL, d);
			if (match.first!=StateMatcher::_TRUE){
				g[v].options = {d};
				g[v].endPose=s_tmp.endPose;
				g[v].Dn=s_tmp.Dn;
				g[first_edge.first].step= gt::distanceToSimStep(g[v].distance(), a.getLinearSpeed());
				first_edge=add_vertex_retro(v, v1,g, g[v].Dn); //passing on the disturbance
				g[v1].Di=g[v].Di;
				g[v1].start=g[v].endPose;
				g[v].phi=NAIVE_PHI;
				g[first_edge.first].direction=d;
				g[v].outcome=simResult::safeForNow;
			}
			else{
				v1=match.second;
			}
			split.push_back(v1);
			nNodes--;
		}
		if (nNodes<=1){
			s_tmp.endPose=endPose;
			std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(s_tmp, g, NULL, d);
			if (match.first!=StateMatcher::_TRUE || match.second==v){
				g[v1].endPose = endPose;
				g[first_edge.first].step= gt::distanceToSimStep(g[v1].distance(), a.getLinearSpeed());	
				g[v1].outcome=simResult::crashed;	
				g[v1].phi=og_phi;
			}

		}

		v=v1;

	}
	return split;
}


void Configurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, const std::set<vertexDescriptor>& closed, TransitionSystem&g, std::vector <vertexDescriptor>& plan){
	for (vertexDescriptor v:evaluation_q){
		std::pair<bool, edgeDescriptor> ep(false, edgeDescriptor());
		std::vector <vertexDescriptor> split = gt::task_vertices(v, g, iteration, currentVertex, &ep); 
		Direction direction= g[ep.second].direction;
		if (split.size()<2){
			split =splitTask(v, g, DEFAULT, ep.second.m_source);
		}
		for (int i=0; i<split.size(); i++){ //
			vertexDescriptor split_v=split[i], src=TransitionSystem::null_vertex();
			if (i<1){
				auto ep=gt::getMostLikely(g, gt::inEdges(g, split_v), iteration);
				if (ep.first){
					src=ep.second.m_source;
				}
			}
			else{
				src=split[i-1];
			}
			EndedResult local_er=estimateCost(g[split_v],g[split_v].start, direction);
			g[split_v].phi=evaluationFunction(local_er);
			applyTransitionMatrix(g, split_v, direction, local_er.ended,src, plan);
			addToPriorityQueue(split_v, priority_q, g, closed);
			src=split_v;
		}
	}
	evaluation_q.clear();

}

void Configurator::propagateD(vertexDescriptor v1, vertexDescriptor v0,TransitionSystem&g, std::vector<vertexDescriptor>*propagated, std::set <vertexDescriptor>*closed,StateMatcher::MATCH_TYPE match){
	if (g[v1].outcome == simResult::successful){
		return;
	}
	vertexDescriptor p=TransitionSystem::null_vertex();
	std::pair <edgeDescriptor, bool> ep= boost::edge(v0, v1, g);
	Disturbance dist = g[v1].Dn;
	if (!ep.second){
		return;
	}
	Direction dir= g[ep.first].direction;
	bool same_Di=g[ep.first.m_source].Di==g[ep.first.m_target].Di;
	ep.first= *(boost::in_edges(ep.first.m_source, g).first);
	ep.second= boost::edge(ep.first.m_source, ep.first.m_target, g).second;
	bool same_direction=gt::check_edge_direction(ep, g, dir) ||( (gt::check_edge_direction(ep, g, STOP))&& dir==DEFAULT) ;
	if (same_direction&& same_Di && g[ep.first.m_target].Dn.getAffIndex()==NONE){
 			g[ep.first.m_target].Dn = dist; //was target
 	}
	if (v1==currentVertex){
		g[v0].outcome=simResult::safeForNow;
	}
	return;
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
		gt::update(e, std::pair <State, Edge>(g[pair.first], g[e2]),g, pair.second==currentVertex, iteration);
		float match_distance=10000;
		for (edgeDescriptor r:toReassign){ //reassigning edges
			auto new_edge= gt::add_edge(r.m_source, pair.second, g, iteration);
			// if (g[r.m_source].visited()){
			// 	EndedResult er=estimateCost(g[pair.second], g[r.m_source].endPose); //reassign cost
			// 	g[pair.second].phi =evaluationFunction(er);	
			// }
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

// void Configurator::clearFromMap(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> matches , TransitionSystem&g, std::unordered_map<State*, ExecutionError>map){
// 		for (std::pair<vertexDescriptor, vertexDescriptor> pair:matches){
// 			if (auto it=map.find(g[pair.first].ID); it!=map.end()){
// 				ExecutionError exer = map.at(transitionSystem[pair.first].ID);
// 				map.insert_or_assign(transitionSystem[pair.second].ID, exer);
// 				map.erase(it);
// 				break;
// 			}

// 		}
// }

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
	auto start_time=std::chrono::high_resolution_clock::now();
	do{
		frontier_v=frontierVertices(src, g, DEFAULT, been);
		// if (src==currentVertex){
		// }
		priorityQueue.erase(priorityQueue.begin());
		for (Frontier f: frontier_v){ //add to priority queue
			//printf("frontier count:%i\n", frontier_v.size());
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
					pend++; //go back a step
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
						path=_p; //
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
			printf("end pose of end vertex:");
			debug::print_pose(g[end].endPose);	
			auto now_time=std::chrono::high_resolution_clock::now();
			std::chrono::duration<float, std::milli>time_elapsed= now_time - start_time;
			//printf("inner loop count= %f\n", time_elapsed.count());
			if (abs(time_elapsed.count()) >100){
				printf("stuck planning, exiting\n");
			break;
		}
		}
//		priorityQueue.erase(priorityQueue.begin());
		for (vertexDescriptor c:add){
			g[c].label=VERTEX_LABEL::UNLABELED;
			path->push_back(c);	
			path_end=c;			
		}
		_finished=overarching_goal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended;
		if (NULL!=finished){
			*finished=_finished;
		}
		if (_finished){
			goal=path_end;
		}
		auto now_time=std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli>time_elapsed= now_time -start_time;
		//printf("outer loop count= %f\n", time_elapsed.count());
		if (abs(time_elapsed.count()) >100){
			printf("stuck planning, exiting\n");
			break;
		}
	}while(!priorityQueue.empty() && (path_end!=goal && !(_finished)));
	auto vs=boost::vertices(g);
	float final_phi=10000;
	for (std::vector<vertexDescriptor> p: paths){
		vertexDescriptor end_plan= *(p.rbegin().base()-1);
		//LAMBDA
		auto skip_first= [](const std::vector<vertexDescriptor> &_plan, const vertexDescriptor & _cv, const TransitionSystem & _g, const int & _step){
			if (_plan.size()==1 && _plan[0]==_cv && _step==0){
				printf("getting whole plan\n");
				return std::vector(_plan.begin()+0, _plan.end());
			}
			else{
				//printf("getting plan from index 1\n");
				printf("plan size before skip %i, first=%i ", _plan.size() , _plan[0]);
				return std::vector((_plan.begin()+1), _plan.end());
			}
		};
		if (end_plan==goal){
			plan=skip_first(p, currentVertex, g, currentTask.motorStep);
			break;
		}
		else if (g[end_plan].phi<final_phi){
			plan=skip_first(p, currentVertex, g, currentTask.motorStep);
			final_phi=g[end_plan].phi;
		}
	}
	printf("PLANNED! size=%i\n",plan.size());
	return plan;

}


void Configurator::skip_reduced(edgeDescriptor& e, TransitionSystem &g, const std::vector<vertexDescriptor> & plan,  std::vector<vertexDescriptor>::iterator it){ 
edgeDescriptor e_start=e;
//adjust here
	do{
		auto ep=boost::edge(*it, *(it+1), g);
		it++;
		if (!ep.second){
			return;
		}
		else{
			e=ep.first;
		}
	}	while(g[e].direction==g[e_start].direction && it != plan.end() && it!=(plan.end()-1)&& g[e].direction==DEFAULT && (g[e.m_target].Di==g[e_start.m_source].Di));
}



// std::vector <vertexDescriptor> Configurator::back_planner(TransitionSystem& g, vertexDescriptor leaf, vertexDescriptor root){
// 	std::vector <vertexDescriptor> vertices;
// 	if (leaf ==root){
// 		throw std::invalid_argument("wrong order of vertices for iteration\n");
// 	}
// 	while(leaf !=root){
// 		if (boost::in_degree(leaf, g)<1){
// 			break;
// 		}
// 		else{
// 			edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
// 			vertexDescriptor src = boost::source(e,g);
// 			// if (g[leaf].endPose != g[src].endPose){ //if the node was successful
// 			if (g[e].step>0){
// 				vertices.insert(vertices.begin(), leaf);
// 			}
// 		leaf = src; //go back
// 		}
// 	}
// 	//vertices.insert(vertices.begin(), root);
// 	return vertices;
// }





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
	std::vector <vertexDescriptor> plan= *p;
	vertexDescriptor pre=currentVertex;
	printf("PLAN:");
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
	//printf("run\n");
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
			if (g[e].it_observed==iteration){ //g[e.m_target].visited()
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
				//in order, try the task which represents the reflex towards the goal
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
	else if (state.outcome==simResult::successful) { //will only enter if successful
		if (d== LEFT || d == RIGHT){
			state.options = {DEFAULT};
			if (src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.disturbance.pose().p)<0){
				state.options.push_back(d);
			}
		}
		else {
			if (src==TransitionSystem::null_vertex()){
				state.options={currentTask.direction};
			}
			else if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
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

void Configurator::applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov){
	if (!g[v0].options.empty()){
		return;
	}
	if (controlGoal.endCriteria.hasEnd()){
		if (ended){
			return;
		}
	}
	else if(round(g[v0].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		return;
	}
	if (src!=movingVertex  && uint(src)<(g.m_vertices.size()-1)&& v0!=movingVertex){ //src< v size is to check that src isn't a garbage value (was giving throuble with tests)
		auto e=boost::edge(src, v0, g); //not adding options to vertices which don't cover a distance unless they're current v
		if (e.second){
			if (g[e.first].step==0){
				return;
			}			
		}
	}
	if (v0==movingVertex || src==TransitionSystem::null_vertex()){
		transitionMatrix(g[v0], DEFAULT, TransitionSystem::null_vertex());	
	}
	else if (auto it =check_vector_for(plan_prov, v0); it!=plan_prov.end() && it!=(plan_prov.end()-1)){
		auto e=boost::edge(src, v0, g);
		skip_reduced(e.first, g, plan_prov, it);
		if ((g[e.first.m_target].visited()&& g[e.first].it_observed<iteration)|| !g[e.first.m_target].visited()){ // 
			g[v0].options={g[e.first].direction};
		}
	}
	else{
		transitionMatrix(g[v0], d, src);
	}
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

void Configurator::adjust_simulated_task(const vertexDescriptor &v, TransitionSystem &g, Task * t){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	if (t->action.direction==currentTask.action.direction){
		t->endCriteria=currentTask.endCriteria;
	}
	else if (t->action.direction==getOppositeDirection(currentTask.direction).second){
		t->setEndCriteria(Angle(M_PI-t->endCriteria.angle.get()));
	}
}



void Configurator::adjust_rw_task(const vertexDescriptor &v, TransitionSystem &g, Task * t, const b2Transform & deltaPose){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	// auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	// int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
	// float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
	// float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
	if (t->getAction().getOmega()!=0){
		float remainingAngle = t->endCriteria.angle.get()-abs(deltaPose.q.GetAngle());
	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
		// if (t->direction==getOppositeDirection(t->direction).second){
		// 	remainingAngle=M_PI-remainingAngle;
		// }
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(t->getAction().getLinearSpeed()>0){
		//step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
		t->setEndCriteria(Distance(t->endCriteria.distance.get()-deltaPose.p.Length()));
	}			// -estimated distance covered

}

std::vector <Frontier> Configurator::frontierVertices(vertexDescriptor v, TransitionSystem& g, Direction d, bool been){
	std::vector <Frontier> result;
	std::pair<edgeDescriptor, bool> ep=boost::edge(movingVertex, v, g); 
	vertexDescriptor v0=v, v1=v, v0_exp;
	//do{
		if ((controlGoal.disturbance.getPosition()-g[v].endPose.p).Length() >= DISTANCE_ERROR_TOLERANCE){
			auto es=boost::out_edges(v, g);
			for (auto ei=es.first; ei!=es.second; ei++){
			std::vector <vertexDescriptor>connecting;
			auto ei2=ei, ei3=ei;
			auto es2=boost::out_edges((*ei).m_target, g);
			auto es3=es2;
			std::vector <vertexDescriptor>connecting2;
			NotSelfEdge not_self_edge(&g);
			do {
				if ((g[(*ei3).m_target].visited() || been)&& not_self_edge(*ei3)){ //(*ei3).m_source!=(*ei3).m_target
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
				else { //not sure if this is right just added
					break;
				}
				if(ei3==es3.second){
					if (ei3!=ei2 && ei2!=ei){
						if (ei2!=es2.second){
							ei2++;
							ei3=ei2;
							es3=es2;						}
					}					
				}
				//printf("is stuck, ei3=%i ->%i\n", (*ei).m_source, (*ei).m_target);
			}while (ei3!=es3.second);
	}
	}

	return result;
}



	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(State s, TransitionSystem& g, State * src, Direction dir, StateMatcher::MATCH_TYPE match_type, std::vector <vertexDescriptor> * others, bool relax,bool wholeTask){
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
		//make state representing a whole task, this is inefficient and when i have time should be susbtituted with subgraph
		State q= g[v];
		if (wholeTask){
			if (auto vertices=gt::task_vertices(v, g, iteration, currentVertex); vertices.size()>1){
				q.start=g[vertices[0]].start;
			}			
		}
		if (v==currentVertex && currentTask.motorStep!=0){
			q.start=b2Transform_zero;
		}
		StateDifference sd(s, q);
		bool condition=0;
		StateMatcher::MATCH_TYPE m=StateMatcher::_FALSE;
		float sum_tmp=sd.get_sum(match_type);
		// if (v==currentVertex){
		// 	printf("Di difference:");
		// 	debug::print_pose(sd.Di.pose);
		// 	printf("Dn difference:");
		// 	debug::print_pose(sd.Dn.pose);
		// }
		if (!relax){
			m=matcher.isMatch(sd, s.endPose.p.Length());
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
	printf("others not null\n");
	for (auto vp:others_set){
		others->push_back(vp.first);
	}
	return result;
}



// std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(vertexDescriptor v, TransitionSystem& g, Direction dir, StateMatcher::MATCH_TYPE match_type, std::vector <vertexDescriptor>* others){
// 	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::_FALSE, TransitionSystem::null_vertex());
// 	auto vs= boost::vertices(g);
// 	//float prob=0;
// 	int nObs=0;
// 	for (auto vi=vs.first; vi!= vs.second; vi++){
// 		if (*vi!=v){
// 			std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
// 			bool Tmatch=true;
// 			//if (dir!=Direction::UNDEFINED){
// 			Tmatch= !ie.empty()||dir==Direction::UNDEFINED;
// 			//}
// 			if (StateMatcher::MATCH_TYPE m=matcher.isMatch(g[v], g[*vi]); matcher.match_equal(m, match_type) && *vi!=movingVertex &Tmatch & boost::in_degree(*vi, g)>=ie.size()){ //
// 				if(g[v].nObs>nObs){
// 				result.first=matcher.isMatch(g[v], g[*vi]);
// 				result.second=*vi;
// 				//prob=g[most_likely.second].probability;
// 				nObs=g[v].nObs;
// 				if (NULL!=others){
// 					others->push_back(result.first);
// 				}
// 			}
// 		}
// 		}
// 	}
// 	return result;
// }

void Configurator::match_setup(bool& closest_match, StateMatcher::MATCH_TYPE& desired_match, const vertexDescriptor& v, std::vector<vertexDescriptor>& plan_prov, const Direction& dir,  TransitionSystem & g){
	if (currentTask.motorStep!=0 || !planVertices.empty() ){ //
	//if (plan_prov.empty()){
		return;
	//}
	}
	auto v_it=check_vector_for(plan_prov, v);
	if ((v==movingVertex || v==currentVertex) || v_it!=plan_prov.end() ){ //|| !plan_prov.empty()
		int out_deg = boost::out_degree(v, g);
		if (g[v].options.capacity() < out_deg || v_it!=plan_prov.end() || gt::inEdges(g, v, STOP).empty()){ //|| gt::inEdges(g, v, STOP).empty()
			desired_match=StateMatcher::MATCH_TYPE::ABSTRACT;
		}
		if (v==currentVertex && dir==currentTask.direction ){ //!plan_prov.empty() || dir==currentTask.direction
			closest_match=true;
		}
	}


}




void Configurator::trackTaskExecution(Task & t){
	b2Transform deltaPose=worldBuilder.wb_bridger.get_transform(&t, data2fp); //track using obstacle OR dead reckoning
	//here can insert something for wb.bridger, wheel speed control (for step)
	adjust_rw_task(movingVertex, transitionSystem, &t, deltaPose); //readjust end criteria
	updateGraph(transitionSystem, deltaPose);//lateral error is hopefully noise and is ignored
	math::applyAffineTrans(deltaPose, t.disturbance); //remove later
	if(t.motorStep==0 || (t.checkEnded()).ended){
		t.change=1;
	}

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
	printf("pv=%i\n", pv.size());
	if (!b){
		boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		return pv;
	}
	if (planning){
		if (pv.empty()){
			printf("I DON'T KNOW WHAT TO DO NOW\n");
			currentTask=Task(controlGoal.disturbance, DEFAULT);
			currentTask.action.L=0;
			currentTask.action.R=0;
			currentTask.change=1;
			currentVertex=movingVertex;
			return pv;
		}
		std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, pv[0], transitionSystem);
		std::pair<edgeDescriptor, bool> ep_mov=boost::edge(movingVertex, pv[0], transitionSystem);
		printf("ep %i -> %i exists=%i, moving exists=%i\n", currentVertex, pv[0], ep.second, ep_mov.second);
		currentVertex= pv[0];
		printf("current v=%i, direction=%s\n", currentVertex, (*dirmap.find(transitionSystem[ep.first].direction)).second);
		pv.erase(pv.begin());
		currentEdge=ep.first;
		transitionSystem[movingVertex].Di=transitionSystem[currentVertex].Di;
		boost::clear_vertex(movingVertex, transitionSystem);
		transitionSystem[movingVertex].outcome=simResult::successful;
		printf("in changeTask: plan size= %i\n", pv.size());
		movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingEdge].direction=transitionSystem[ep.first].direction;
		transitionSystem[movingEdge].step=currentTask.motorStep;
		currentTask = task_to_execute(transitionSystem, currentEdge);
		if (currentTask.action.getLinearSpeed()==0){
			currentTask.motorStep=transitionSystem[currentEdge].step;
		}
		else{
			currentTask.motorStep = gt::distanceToSimStep(transitionSystem[currentVertex].distance(), currentTask.action.getLinearSpeed());// 			
		}
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
		printf("changed to %f\n", currentTask.action.getOmega());
	}
	ogStep = currentTask.motorStep;
	return pv;
}

// void Configurator::trackDisturbance(b2Transform & pose, Task::Action a, float error){
// 	float angleTurned =MOTOR_CALLBACK*a.getOmega();
// 	pose.q.Set(pose.q.GetAngle()-angleTurned);	
// 	float distanceTraversed = 0;
// 	float initialL = pose.p.Length();
// 	if(fabs(error)<TRACKING_ERROR_TOLERANCE){
// 		distanceTraversed= MOTOR_CALLBACK*a.getLinearSpeed();
// 	}
// 	else{
// 		distanceTraversed=error;
// 	}
// 	pose.p.x=cos(pose.q.GetAngle())*initialL-cos(angleTurned)*distanceTraversed;
// 	pose.p.y = sin(pose.q.GetAngle())*initialL-sin(angleTurned)*distanceTraversed;
// }



void Configurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:planVertices){
		if (p==v){
       		g[v].phi-=.1;
			break;
		}
    } 
}

void Configurator::updateGraph(TransitionSystem&g, const b2Transform & deltaPose){
	math::applyAffineTrans(deltaPose, g);
	math::applyAffineTrans(-deltaPose, &controlGoal);
	math::applyAffineTrans(-deltaPose, getTask()->start); //d update happens in get_transform
}

float Configurator::approximate_angle(const float & angle, const Direction & d, const simResult::resultType & outcome){
	float result=angle, decimal, integer;
;
	if ((d==LEFT || d==RIGHT)&& outcome!=simResult::crashed){
		float ratio= angle/ANGLE_RESOLUTION;
		decimal=std::modf(ratio, &integer);
		if (fabs(decimal)>=0.5){
			if (integer<0){
				integer-=1;
			}
			else{
				integer+=1;
			}
		}		
		result=integer*ANGLE_RESOLUTION;
	}
	return result;
}


void Configurator::ts_cleanup(TransitionSystem * g){
	Connected connected(g);
	NotSelfEdge nse(g);
	FilteredTS fts(*g, nse, connected); //boost::keep_all()
	TransitionSystem tmp;
	boost::copy_graph(fts, tmp);
	g->clear();
	g->swap(tmp);		


}
 
void Configurator::shift_states(TransitionSystem & g, const std::vector<vertexDescriptor>& p, const b2Transform & shift_start){
	if (p.empty()){
		return;
	}
	for (const vertexDescriptor &v:p){
		math::applyAffineTrans(shift_start, g[v]);
	}
}
