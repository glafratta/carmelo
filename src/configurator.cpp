#include "configurator.h"
#include <chrono>


void math::applyAffineTrans(const b2Transform& deltaPose, Task* task){
	math::applyAffineTrans(-deltaPose, task->start);
	// if (task.disturbance.getAffIndex()!=NONE){
	// 	math::applyAffineTrans(deltaPose, task.disturbance.bf.pose);
	// }
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
		//transitionSystem[currentVertex].direction=STOP;
	currentTask=Task(Direction::STOP);
//	printf("currentTask l=%f, r=%f\n", currentTask.action.L, currentTask.action.R);
	movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
	currentEdge = boost::add_edge(src, currentVertex, transitionSystem).first;
	printf("dummy, current edge = %i, %i\n", src, currentVertex);
	transitionSystem[movingEdge].direction=STOP;
	transitionSystem[currentEdge].direction=STOP;
	//transitionSystem[currentEdge].it_observed=iteration;
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
	if (data2fp.empty()){
		printf("data empty!\n");
		return 1;
	}
	//currentBox2D = CoordinateContainer(data2fp);
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
	if (planning){ //|| !planError.m_vertices.empty())
		// auto checkedge = boost::edge(movingVertex, currentVertex, transitionSystem);
		// transitionSystem[movingVertex].Dn=transitionSystem[currentVertex].Dn;
		// transitionSystem[movingVertex].Dn.invalidate();
		// if (debugOn){
		// 	printf("moving edge= %i -> %i\n", movingEdge.m_source, movingEdge.m_target);
		// }
		// //have I seen this envronment configuration before?
		//  vertexDescriptor src; //ve=TransitionSystem::null_vertex(),
		// if (!planVertices.empty()){
		// 	//ve= *(planVertices.rbegin().base()-1);
		// 	src=movingVertex;
		// }
		// else {
		// 	//ve=currentVertex;
		// 	src=currentVertex;
		// }
	// bool 	plan_works=false;
	// // }
	 std::vector <std::pair <vertexDescriptor, vertexDescriptor>> toRemove;
//	if (!plan_works){	// boost::out_degree(src, transitionSystem) <1		
		//boost::clear_vertex(movingVertex, transitionSystem);
		if (transitionSystem.m_vertices.size()==1){
			dummy_vertex(currentVertex);//currentEdge.m_source
			currentTask.change=1;
		}
		vertexDescriptor src; //ve=TransitionSystem::null_vertex(),
		if (!planVertices.empty() && currentTask.motorStep!=0){
			src=movingVertex;
		}
		else {
			src=currentVertex;
		}
		resetPhi(transitionSystem);
		toRemove=explorer(src, transitionSystem, currentTask, world);
		clearFromMap(toRemove, transitionSystem, errorMap);
		ts_cleanup(&transitionSystem);
		if (debugOn){
			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, planVertices, currentVertex);
		}
		planVertices= planner(transitionSystem, src);
		if (debugOn){
			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, planVertices, currentVertex);
		}
		printPlan();
		boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		explored=1;
//	}


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
	//printf("took %f seconds\n", duration);
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
Disturbance Configurator::getDisturbance(TransitionSystem&g, const  vertexDescriptor& v, b2World & world, const Direction& dir){
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
						return g[v].Di;
					}
				}
				//check if Di was eliminated 
				return controlGoal.disturbance;
			}
			else if (v==movingVertex){
			return g[v].Di;
			}
	}
	return g[v].Dn;
	//return controlGoal.disturbance;
}


Task Configurator::task_to_execute(const TransitionSystem & g, const edgeDescriptor& e){
	Task t=controlGoal;
	// if (Disturbance Dn= g[e.m_target].Dn; Dn.getAffIndex()==AVOID){
	// 	Disturbance Di= Dn;
	// 	Di.affordanceIndex=PURSUE;
	// 	t=Task(Di, g[e].direction, b2Transform_zero, true);
	// 	float distance = g[e.m_target].end_from_disturbance().p.Length();
	// 	t.setEndCriteria(Distance(distance));
	// }
	// else{
		t=Task(g[e.m_target].Di, g[e].direction, b2Transform_zero, true);

	//}
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
	result =t.willCollide(w, iteration, robot.body, debugOn, remaining, _simulationStep); //default start from 0
	//approximate angle to avoid stupid rounding errors
	float approximated_angle=approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode);
	result.endPose.q.Set(approximated_angle);
	return result;
	}


std::vector <std::pair<vertexDescriptor, vertexDescriptor>>Configurator::explorer(vertexDescriptor v, TransitionSystem& g, Task t, b2World & w){
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	Direction direction=currentTask.direction;
	std::vector <vertexDescriptor> priorityQueue = {v}, evaluationQueue, plan_prov=planVertices;
	std::set <vertexDescriptor> closed;
	b2Transform start= b2Transform_zero, shift=b2Transform_zero;
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> toRemove;
	do{
		v=bestNext;
		closed.emplace(*priorityQueue.begin().base());
		priorityQueue.erase(priorityQueue.begin());
		EndedResult er = controlGoal.checkEnded(g[v], t.direction);
		// if (er.ended){
		// 	break;
		// }
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
				changeStart(start, v0, g, shift);
				Disturbance Di=getDisturbance(g, v0, w, g[v0].options[0]);
				//worldBuilder.world_cleanup(&w);
				t = Task(Di, g[v0].options[0], start, true);
				std::pair <State, Edge> sk(State(start, Di), Edge(g[v0].options[0]));
				float _simulationStep=BOX2DRANGE;
				adjustStepDistance(v0, g, &t, _simulationStep);
				worldBuilder.buildWorld(w, data2fp, t.start, t.direction, t.disturbance, 0.15, WorldBuilder::PARTITION); //was g[v].endPose
				simResult sim=simulate(t, w, _simulationStep); //sk.first, g[v0], 
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.direction=t.direction;
				sk.second.it_observed=iteration;
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
				bool closest_match=false, match_task=true;
				StateMatcher::MATCH_TYPE desired_match=StateMatcher::MATCH_TYPE::ABSTRACT;
				//match_setup(closest_match, desired_match, v0, plan_prov, t.direction, g);
				std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(sk.first, g, source, t.direction, desired_match, NULL, closest_match, match_task );		//, closest_match	
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher.match_equal(match.first, desired_match)){
					g[v0].options.erase(g[v0].options.begin());
					v1=match.second; //frontier
					//if ((v0!=v1)){
						edge= gt::add_edge(v0, v1, g, iteration, t.direction); //assumes edge added
						//new_edge=edge;
						//edge.second=true; //just means that the edge is valid
						if (edge.second){
							g[edge.first]=sk.second; //doesn't update motorstep
						}
					//}
					if (plan_prov.empty()&&currentTask.motorStep==0){
						bool finished=false, been=matcher.match_equal(match.first, StateMatcher::ABSTRACT); //(match.first==StateMatcher::DISTURBANCE); //ADD representation of task but shifted
						//std::pair<bool, edgeDescriptor> e_tmp(edge.second, edge.first);
						std::vector <vertexDescriptor> task_vertices=gt::task_vertices(v1, g, iteration, currentVertex);
						vertexDescriptor task_start= task_vertices[0];
						Task controlGoal_adjusted= controlGoal;
						math::applyAffineTrans(-g[task_start].start, &controlGoal_adjusted);
						auto plan_tmp=planner(g, task_start, TransitionSystem::null_vertex(), been, &controlGoal_adjusted, &finished);
						bool filler=0;
						shift= start-g[task_start].start;
						if (finished){
							plan_prov=plan_tmp;
							boost::remove_edge(edge.first, g);
							edge= gt::add_edge(v0, task_start, g, iteration, g[edge.first].direction);
							//edge=new_edge;
							if (t.direction== g[edge.first].direction){
								g[v0].options.clear();
							}
							else{
								g[v0].options={g[edge.first].direction};
							}
						}
					}
				}
				else{
					edge= add_vertex_now(v0, v1,g,sk.first.Di, sk.second); //addVertex
					g[edge.first.m_target].label=sk.first.label; //new edge, valid
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, errorMap, iteration);
					gt::adjustProbability(g, edge.first); //new_edge to allow to adjust prob if the sim state has been previously ecountered and split
					// if (edge.first!= new_edge.first && new_edge.first!=edgeDescriptor()){
					// 	boost::remove_edge(edge.first, g);
					// }
				}
				applyTransitionMatrix(g, v1, t.direction, er.ended, v0, plan_prov);
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
	if (controlGoal.getAffIndex()==PURSUE){
			//printf("best=%i, options=%i\n", bestNext, g[bestNext].options);
	}
	std::vector <edgeDescriptor> best_in_edges= gt::inEdges(g,bestNext);
	if (best_in_edges.empty()){
		direction=currentTask.direction;
	}
	else{
	direction = g[best_in_edges[0]].direction;
	}
}while(g[bestNext].options.size()>0);
printf("finished exploring\n");
return toRemove;
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
	//if (src!=TransitionSystem::null_vertex()){
	if (auto ie=gt::inEdges(g, src, DEFAULT), stop_edges=gt::inEdges(g, src, STOP); !ie.empty()|| !stop_edges.empty()){
		split.insert(split.begin(), src);
		g[src].outcome=simResult::safeForNow;
	}
	//}
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


void Configurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, const std::set<vertexDescriptor>& closed, TransitionSystem&g){
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
			applyTransitionMatrix(g, split_v, direction, local_er.ended,src, planVertices);
			addToPriorityQueue(split_v, priority_q, g, closed);
			src=split_v;
		}
	}
	evaluation_q.clear();

}

std::vector<std::pair<vertexDescriptor, vertexDescriptor>> Configurator::propagateD(vertexDescriptor v1, vertexDescriptor v0,TransitionSystem&g, std::vector<vertexDescriptor>*propagated, std::set <vertexDescriptor>*closed,StateMatcher::MATCH_TYPE match){
	std::vector<std::pair<vertexDescriptor, vertexDescriptor>> deletion;
	if (g[v1].outcome == simResult::successful){
		return deletion;
	}
	vertexDescriptor p=TransitionSystem::null_vertex();
	std::pair <edgeDescriptor, bool> ep= boost::edge(v0, v1, g);
	Disturbance dist = g[v1].Dn;
	//while (ep.second){
	if (!ep.second){
		return deletion;
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

void Configurator::clearFromMap(std::vector<std::pair<vertexDescriptor, vertexDescriptor>> matches , TransitionSystem&g, std::unordered_map<State*, ExecutionError>map){
	//auto es=boost::edges(g);
	//for (auto ei=es.first; ei!=es.second; ei++){
		for (std::pair<vertexDescriptor, vertexDescriptor> pair:matches){
			if (auto it=map.find(g[pair.first].ID); it!=map.end()){
				ExecutionError exer = map.at(transitionSystem[pair.first].ID);
				map.insert_or_assign(transitionSystem[pair.second].ID, exer);
				map.erase(it);
				break;
			}

		}
	//}
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
				//printf("new empty path\n");
				break;
			}
			//printf("inner loop v %i \t", end);
			debug::print_pose(g[end].endPose);	
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
		//printf("outer loop\n");
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
	//printf("PLANNED!\n");
	// if (_finished && g[plan[plan.size()-1]].Dn.getAffIndex()==NONE){
	// 	g[plan[plan.size()-1]].Dn=controlGoal.disturbance;
	// 	g[plan[plan.size()-1]].Dn.invalidate();
	// }
	return plan;

}


bool Configurator::checkPlan(b2World& world, std::vector <vertexDescriptor> &p, TransitionSystem &g, Disturbance & dist_match, b2Transform start, vertexDescriptor custom_start){
	b2Vec2 v = controlGoal.disturbance.getPosition() - b2Vec2(0,0);
	//printf("check goal start: %f, %f, %f, distance = %f, valid =%i\n", controlGoal.start.p.x,controlGoal.start.p.y, controlGoal.start.q.GetAngle(), v.Length(), controlGoal.disturbance.isValid());
	//printf("CHECK goal position= %f, %f, %f, valid =%i\n", controlGoal.disturbance.pose().p.x, controlGoal.disturbance.pose().p.y, controlGoal.disturbance.pose().q.GetAngle(), controlGoal.disturbance.isValid());
	bool result=true;
	int it=0, end_it=p.size();//this represents currentv
	//printPlan();
	std::vector <vertexDescriptor> vpt=p;
	if (p.empty() && currentTask.motorStep==0){
		return false;
	}
	printf("CHECKING\n");
	if (p.size()>0){
		if (p[0]==currentVertex){
			p.erase(p.begin());
		}		
	}	
	if (custom_start==TransitionSystem::null_vertex()){
		custom_start=movingVertex;
		it=-1;
//		end_it=p.size(); 
	}
	std::pair<edgeDescriptor, bool> ep(edgeDescriptor(), false);
	if (currentTask.motorStep==0){
		ep =boost::edge(custom_start, *p.begin(), g);	
	} 
	else{
		ep =boost::edge(custom_start, currentVertex, g);	

	}
	printf("start from v=%i\n", ep.first.m_target);
	std::pair <State, Edge> sk;
	b2Transform shift=b2Transform_zero, compare_start=b2Transform_zero;
	if (SignedVectorLength(g[ep.first.m_target].start.p)<0  && SignedVectorLength(g[p[p.size()-1]].endPose.p)<0){
		shift=g[ep.first.m_target].start-g[movingVertex].start;
	}
	do {
//printf("start= \t");
		//debug::print_pose(start);
		//Disturbance d_adjusted=getDisturbance(g, ep.first.m_source,world, g[ep.first].direction);
		vertexDescriptor t_start_v=ep.first.m_target; //vertex denoting start of task
		Disturbance d_adjusted;
		bool reset_end_criteria=0;
		sk.first=State(start);
		sk.first.Dn=dist_match;
		State compare_tmp=g[t_start_v];
		if (int(t_start_v)==int(currentVertex) && g[t_start_v].start.p.x<0){
			compare_start=b2Transform_zero-shift;
		}
		else{
			compare_start=g[t_start_v].start;	
		}
		compare_tmp.start=compare_start;
		StateDifference sd_check;
		auto state_match=matcher.isMatch(compare_tmp,sk.first, NULL, &sd_check);
		if (state_match==StateMatcher::ABSTRACT){
		//if (g[t_start_v].Dn.affordanceIndex==AVOID){
			//d_adjusted=g[t_start_v].Dn;
			d_adjusted=dist_match;
			d_adjusted.validate();
			d_adjusted.affordanceIndex=PURSUE;
			reset_end_criteria=1;
		}
		else{
			d_adjusted=g[t_start_v].Di;
			math::applyAffineTrans(shift, d_adjusted);
		}
		Task t= Task(d_adjusted, g[ep.first].direction, start, true);
		float stepDistance=BOX2DRANGE;
		worldBuilder.buildWorld(world, data2fp, start, t.direction, t.disturbance, 0.15, WorldBuilder::CLUSTERING::PARTITION);
		sk.first.Di=d_adjusted;
		sk.second=Edge(t.direction);
		//printf("from %i", ep.first.m_target);
		b2Transform endPose= skip(ep.first,g,it, &t, stepDistance, p);
		if (reset_end_criteria){
			float distance = g[ep.first.m_source].end_from_Dn().p.Length();
			t.setEndCriteria(Distance(distance));
		}
		compare_tmp=g[ep.first.m_source];
		if (int(t_start_v)==int(currentVertex) && g[t_start_v].start.p.x<0){
			compare_start=b2Transform_zero;
		}
		else{
			compare_start=g[t_start_v].start;	
		}
		compare_tmp.start=compare_start;
		simResult sr = simulate( t, world); //sk.first, g[p[it-1]],
		if (sr.resultCode==simResult::crashed){
			return false;
		}
		gt::fill(sr, &sk.first, &sk.second); //this also takes an edge, but it'd set the step to the whole
									// simulation result step, so this needs to be adjusted
		//COMMENITNG THIS BIT OUT BECAUSE NOW WE DON'T CARE ABOUT FIXED DISTANCE
		b2Transform expected_deltaPose=(endPose-g[t_start_v].start);
		start = sk.first.endPose;
		StateDifference sd;
		bool match_outcome=true;
		StateMatcher::MATCH_TYPE is_match=matcher.isMatch(compare_tmp, sk.first, NULL, &sd, match_outcome); //og match to g[ep.first.m_source]
		vertexDescriptor v1=ep.first.m_source;
		std::pair <bool,edgeDescriptor> prev_edge= gt::getMostLikely(g, gt::inEdges(g, v1, t.direction),iteration);
		if (!prev_edge.first){
			auto _moving=boost::add_edge(movingVertex, v1, g);
			prev_edge.second=_moving.first;
		}
		if (!matcher.match_equal(is_match, StateMatcher::DN_POSE)){
			result=false;
			// printf("MISMATCH!!! with v %i expected D:", ep.first.m_source);
			// debug::print_pose(g[ep.first.m_source].Dn.bf.pose);
			// printf("start:");
			// debug::print_pose(g[t_start_v].start);
			// printf(", w=%f, l=%f\n", g[ep.first.m_source].disturbance.bf.halfWidth, g[ep.first.m_source].disturbance.bf.halfLength);
			//printf("observed D:");
			//debug::print_pose(sk.first.disturbance.bf.pose);
			//printf(", w=%f, l=%f\n", sk.first.disturbance.bf.halfWidth, sk.first.disturbance.bf.halfLength);
			return result;
		}
		// else{
		// 	gt::update(prev_edge.second, sk, g,true, errorMap, iteration);
		// }
		//propagateD(v1,prev_edge.second.m_source, g);
		//gt::adjustProbability(g, ep.first);
		//printf("skipping from %i, edge %i ->%i\n", it, ep.first.m_source, ep.first.m_target);
	}while (ep.first.m_target!=TransitionSystem::null_vertex() && it <end_it && result );
	// if (ep.first.m_target==TransitionSystem::null_vertex()){
	// }
	// else if (it >=end_it){
	// 	result=controlGoal.checkEnded(g[ep.first.m_target]).ended;
	// }
	result=controlGoal.checkEnded(sk.first,UNDEFINED, true).ended;

	return result;
}


b2Transform Configurator::skip(edgeDescriptor& e, TransitionSystem &g, int& i, Task* t, float& step, std::vector <vertexDescriptor> plan){ 
	b2Transform result;
	edgeDescriptor e_start=e;

	// if (e_start==movingEdge){
	// 	e_start=currentEdge;
	// }
	vertexDescriptor v_tgt= e.m_target;
	Disturbance D=g[v_tgt].Di;
//adjust here
	do{
		i++;
		printf("index=%i\n", i);
		//printf("iterator = %i, e src= %i, e trgt= %i\n", i, e.m_source, e.m_target);
			auto es = boost::out_edges(e.m_target, g);
			g[e].it_observed=iteration;
			if (es.first==es.second){
				vertexDescriptor new_src=e.m_target;
				e.m_source=new_src;
				e.m_target=TransitionSystem::null_vertex();
			}
			for (auto ei = es.first; ei!=es.second; ++ei){ //was ++ei
				if (plan.empty()){
					break;
				}
				if ((*ei).m_target == plan[i]){
					e= (*ei);
					v_tgt=e.m_source;
					break;
				}
			}
		result=g[e.m_source].endPose; 
		}while (g[e].direction==t->direction && i<planVertices.size()&& g[e].direction==DEFAULT && (g[e.m_target].Di==D));
//	printf("ended skip, result = %f, %f, %f\n", result.p.x, result.p.y, result.q.GetAngle());
	if (g[e_start.m_target].Dn.getAffIndex()!=NONE){
		step=b2Vec2(g[e_start.m_source].endPose.p-g[e_start.m_target].Dn.pose().p).Length();
	}
	else{
		if (g[e_start].direction==DEFAULT){
			//step = (g[e_start.m_source].endPose.p- g[v_tgt].endPose.p).Length();
			printf("step=%f\n", step);
		}
		else{
	 	adjustStepDistance(e_start.m_source,g, t, step, std::pair(true,v_tgt));
		}
		printf("adjusted step=%f\n", step);
	}

	return result;
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



std::vector <vertexDescriptor> Configurator::back_planner(TransitionSystem& g, vertexDescriptor leaf, vertexDescriptor root){
	std::vector <vertexDescriptor> vertices;
	if (leaf ==root){
		throw std::invalid_argument("wrong order of vertices for iteration\n");
	}
	while(leaf !=root){
		if (boost::in_degree(leaf, g)<1){
			break;
		}
		else{
			edgeDescriptor e = boost::in_edges(leaf, g).first.dereference(); //get edge
			vertexDescriptor src = boost::source(e,g);
			// if (g[leaf].endPose != g[src].endPose){ //if the node was successful
			if (g[e].step>0){
				vertices.insert(vertices.begin(), leaf);
			}
		leaf = src; //go back
		}
	}
	//vertices.insert(vertices.begin(), root);
	return vertices;
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
	//printf("current=%i\t", pre);
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
	//switch (numberOfM){
	//	case (THREE_M):{
	srand(unsigned(time(NULL)));
	if (state.outcome == simResult::safeForNow){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
			//if (state.nodesInSameSpot<maxNodesOnSpot){
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
			// }
			// else{
			// 	int boo=0;
			// }
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
			// else if (src==TransitionSystem::null_vertex()){
			// 	state.options={currentTask.direction};
			// }
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
	if (v0==movingVertex || src==TransitionSystem::null_vertex()){
		transitionMatrix(g[v0], DEFAULT, TransitionSystem::null_vertex());	
		//return;
	}
	else if (auto it =check_vector_for(plan_prov, v0); it!=plan_prov.end() && it!=(plan_prov.end()-1)){
		//auto e =boost::edge(v0, *(it+1), g); //assuming there is an edge!
		auto e=boost::edge(src, v0, g);
	//	if (v0!=currentVertex){
			skip_reduced(e.first, g, plan_prov, it);
	//	}
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

// std::pair <bool, vertexDescriptor> Configurator::been_there(TransitionSystem & g, Disturbance target){
// 	printf("ENTERED BEEN\n");
// 	std::pair <bool, vertexDescriptor> result(0, TransitionSystem::null_vertex());

// 	std::pair <float, vertexDescriptor> best(10000, TransitionSystem::null_vertex());
// 	float best_prob=0;
// 	auto vs=boost::vertices(g);
// 	for (auto vi=vs.first; vi!=vs.second; vi++ ){
// 		// if (target.getAffIndex()==PURSUE){
// 		// 	//smallest difference to target position
// 		// 	// sd.D_position.x= g[*vi].endPose.p.x-target.pose().p.x;
// 		// 	// sd.D_position.y =g[*vi].endPose.p.y-target.pose().p.y;
// 		// 	// sd.D_angle = angle_subtract(g[*vi].endPose.q.GetAngle(), target.pose().q.GetAngle());
// 		// }
// 		if (target.getAffIndex()==NONE){
// 			//b2Transform reference;
// 			b2Vec2 reference = g[movingVertex].endPose.p-controlGoal.start.p;
// 			//reference.q.Set(reference.q.GetAngle()- g[movingVertex].endPose.q.GetAngle());
// 			float remaining= BOX2DRANGE-SignedVectorLength(reference);
// 			target.bf.pose.p.x=remaining *cos(g[*vi].endPose.q.GetAngle());
// 			target.bf.pose.p.y=remaining*sin(g[*vi].endPose.q.GetAngle());
// 			target.bf.pose.q.Set((g[*vi].endPose.q.GetAngle()));
// 			// sd.r_position.x=g[*vi].endPose.p.x-target_pose.p.x;
// 			// sd.r_position.y =g[*vi].endPose.p.y-target_pose.p.y;
// 			// sd.r_angle =g[*vi].endPose.q.GetAngle()-target_pose.q.GetAngle();
// 		}
// 		State s_target;
// 		s_target.disturbance=target;
// 		StateDifference sd(g[*vi], s_target);
// 		float sum=sd.sum_d_pos();
// 		auto in_edges = gt::inEdges(g, *vi);
// 		auto most_likely_edge=gt::getMostLikely(g, in_edges, iteration);
// 		printf("got likely edge of %i, %i, %i -> %i tot edges =%i\n",*vi, most_likely_edge.first, most_likely_edge.second.m_source, most_likely_edge.second.m_target, in_edges.size());
// 		float prob= 0;
// 		if (most_likely_edge.first ){
// 			prob=g[most_likely_edge.second].probability;//.weighted_probability(iteration);
// 			printf("set prob for %i:  %f\n", *vi, prob);
// 		}
// 		if (StateMatcher::StateMatch sm(sd, matcher.error); sm.disturbance_exact() & (sum<best.first || (sum==best.first & prob>best_prob))){
// 				best.first=sum;
// 				best.second=*vi;
// 				best_prob=prob;
// 				result.first=true;
// 			}
// 	}
// 	result.second=best.second;
// 	//printf("goal =%f, %f, %f", target.pose().p.x, target.pose().p.y, target.pose().q.GetAngle());
// 	printf("been=%i, vertex=%i, pose: %f, %f, %f\n", result.first, result.second, g[result.second].endPose.p.x,g[result.second].endPose.p.y, g[result.second].endPose.q.GetAngle() );
// 	return result; //if been there, do not explore, extract a plan then check it
// }



// std::vector <Pointf> Configurator::neighbours(b2Vec2 pos, float radius){ //more accurate orientation
// 	std::vector <Pointf> result;
// 	cv::Rect2f rect(pos.x-radius, pos.y-radius, radius*2, radius*2);//tl, br, w, h
// 	auto br=rect.br();
// 	auto tl=rect.tl();
// 	for (Pointf p: sensorTools.previous){
// 		if (p.inside(rect) & p!=getPointf(pos)){
// 			result.push_back(p);
// 		}
// 	}
// 	return result;
// }


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

	// if (tgt.first & !g[v].disturbance.isValid()){
	// 	step = (g[v].endPose.p- g[tgt.second].endPose.p).Length();
	// }
	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
	float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
	float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
	if (currentTask.getAction().getOmega()!=0){
		float remainingAngle = currentTask.endCriteria.angle.get()-abs(theta_obs);
	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
		if (t->direction==getOppositeDirection(currentTask.direction).second){
			remainingAngle=M_PI-remainingAngle;
		}
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(currentTask.getAction().getLinearSpeed()>0){
		step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
	}			// -estimated distance covered
	//printf("adjusted\n");
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
				if ((g[(*ei3).m_target].visited() || been)&& (*ei3).m_source!=(*ei3).m_target){
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
					if (ei3!=ei2 & ei2!=ei){
						ei2++;
						ei3=ei2;
						es3=es2;
					}					
				}
				//printf("is stuck, ei3=%i ->%i\n", (*ei).m_source, (*ei).m_target);
			}while (ei3!=es3.second);
	}
	}

	return result;
}

void Configurator::recall_plan_from(const vertexDescriptor& v, TransitionSystem & g, b2World &world,  std::vector <vertexDescriptor>& plan_provisional, bool & plan_works, Disturbance *dist){
   // printf("recalling\n");
	auto srcs= gt::inEdges(g, v);
    vertexDescriptor src=v;
    if(!srcs.empty()){
		src= srcs[0].m_source;
	}
	b2Transform o_shift= -g[v].start;
	printf("shift\n");
	//debug::print_pose(o_shift);
	Task controlGoal_adjusted= controlGoal;
	math::applyAffineTrans(o_shift, &controlGoal_adjusted);
	bool ctrl_finished=false;
	plan_provisional=planner(g, src, TransitionSystem::null_vertex(), false, &controlGoal_adjusted, &ctrl_finished); //been.second, been.first
	//printf("provisional plan from v%i\n", v);
	printPlan(&plan_provisional);
	auto vi= (plan_provisional.end()-1);
	vertexDescriptor end =*(vi);
	if (ctrl_finished){
		plan_works= checkPlan(world, plan_provisional, g, *dist,  g[movingVertex].start,src);
		if (plan_works){
			return;
		}
	}
	plan_provisional.clear();

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
		StateDifference sd(s, q);
		bool condition=0;
		StateMatcher::MATCH_TYPE m=StateMatcher::_FALSE;
		float sum_tmp=sd.get_sum(match_type);
		if (!relax){
			//m= matcher.isMatch(s, g[v], src, &sd);
			m=matcher.isMatch(sd, s.endPose.p.Length());
			condition=matcher.match_equal(m, match_type);
		}
		else{
			condition= sum_tmp<sum;
		}
		if ( condition&& v!=movingVertex && boost::in_degree(v, g)>0 &&Tmatch ){ 
			sum=sum_tmp;
			//std::pair<bool, edgeDescriptor> most_likely=gt::getMostLikely(g, ie, iteration);
			if (NULL!=others){
			 	others_set.emplace(std::pair< vertexDescriptor, float>(v, sum));
			 }
			// if (g[most_likely.second].it_observed<1){
				
			// }
			// else if (most_likely.first ){ //&&g[most_likely.second].probability>prob
				if (!relax){
					result.first= m;
				}
				else{
					result.first=match_type;
				}
				result.second=v;
				//prob=g[most_likely.second].probability;
				
			//}
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

// std::pair <bool, vertexDescriptor> Configurator::exactPolicyMatch(State s, TransitionSystem& g, Direction d){
// 	std::pair <bool, vertexDescriptor> result(false, TransitionSystem::null_vertex());
// 	auto vs= boost::vertices(g);
// 	for (auto vi=vs.first; vi!= vs.second; vi++){
// 		vertexDescriptor v=*vi;
// 		if (matcher.isPerfectMatch(g[v], s) & v!=movingVertex & inEdges(g, v, d).empty()){
// 			result.first=true;
// 			result.second=v;
// 			break;
// 		}

// 	}
// 	return result;
// }

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


void Configurator::changeStart(b2Transform& start, vertexDescriptor v, TransitionSystem& g, const b2Transform& shift){
	if (g[v].outcome == simResult::crashed && boost::in_degree(v, g)>0){
		edgeDescriptor e = boost::in_edges(v, g).first.dereference();
		start = g[e.m_source].endPose + shift;
	}
	else{
		start=g[v].endPose +shift;
	}
}




ExecutionError Configurator::trackTaskExecution(Task & t){
	ExecutionError error;
	//sensor: get points of disturbance
	//get min area rect
	//compare pos/orientation	

	//STEP STUFF, TO BE DELETED
	if (t.motorStep>0 & fabs(error.r())<TRACKING_ERROR_TOLERANCE & fabs(error.theta())<TRACKING_ANGLE_TOLERANCE){
		t.motorStep--;
		//printf("step =%i\n", t.motorStep);
	}
	else if (fabs(error.r())>=TRACKING_ERROR_TOLERANCE){
		int correction=-std::floor(error.r()/(t.action.getLinearSpeed()*LIDAR_SAMPLING_RATE)+0.5);
		t.motorStep+=correction; //reflex
	}
	else if (fabs(error.theta())>=TRACKING_ANGLE_TOLERANCE & t.action.getOmega()!=0){
		int correction=-std::floor(error.theta()/(t.action.getOmega()*MOTOR_CALLBACK)+0.5);
		t.motorStep+=correction; //reflex
	}		

	updateGraph(transitionSystem);//lateral error is hopefully noise and is ignored
	//printf("deltapose= %f, %f, %f\n", deltaPose.p.x, deltaPose.p.y, deltaPose.q.GetAngle());
	if(t.motorStep==0){
		t.change=1;
	}
	// else{
	// 	error.setTheta(taskRotationError()); //will be rest at the next callback
	// }
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
	//printf("currentTask change= %i\n", currentTask.change);
	if (!b){
		return pv;
	}
	if (planning){
		if (pv.empty()){
			//if (currentVertex!=movingVertex){
				printf("I DON'T KNOW WHAT TO DO NOW\n");
				currentTask=Task(STOP);
			//}
			//printf("no plan, bas\n");
			return pv;
		}
		if (movingEdge==edgeDescriptor()){
			//printf("moving edge is null\n");
		}
		if (movingEdge.m_target==TransitionSystem::null_vertex()){
			//printf("null current\n");
		}
		//printf("change plan\n");
		std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, pv[0], transitionSystem);
	//	printf("ep exists=%i, src=%i, tgt=%i\n", !ep.second, ep.first.m_source, ep.first.m_target);
		currentVertex= pv[0];
		pv.erase(pv.begin());
		currentEdge=ep.first;
		transitionSystem[movingVertex].Di=transitionSystem[currentVertex].Di;
		boost::clear_vertex(movingVertex, transitionSystem);
		transitionSystem[movingVertex].outcome=simResult::successful;
	//	printf("changed current %i + cleared 0\n", currentVertex);
		movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingEdge].direction=transitionSystem[ep.first].direction;
		transitionSystem[movingEdge].step=currentTask.motorStep;
		//currentTask = Task(transitionSystem[currentEdge.m_source].Dn, transitionSystem[currentEdge].direction, b2Transform(b2Vec2(0,0), b2Rot(0)), true);
		currentTask = task_to_execute(transitionSystem, currentEdge);
		if (currentTask.action.getLinearSpeed()==0){
			currentTask.motorStep=transitionSystem[currentEdge].step;
		}
		else{
			currentTask.motorStep = gt::distanceToSimStep(transitionSystem[currentVertex].distance(), currentTask.action.getLinearSpeed());// 			
		}
	//	printf("l=%f, r=%f, step=%i\n", currentTask.action.L, currentTask.action.R, currentTask.motorStep);
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

void Configurator::track_disturbance_cl(b2Transform & pose, Task::Action a, float error){

}


void Configurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:planVertices){
		if (p==v){
       		g[v].phi-=.1;
			break;
		}
    } 
}

void Configurator::updateGraph(TransitionSystem&g, b2Transform * _deltaPose){
	b2Transform deltaPose;
	if (NULL==_deltaPose){
		float angularDisplacement= getTask()->getAction().getOmega()*MOTOR_CALLBACK;
		float xdistance=cos(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
		float ydistance=sin(angularDisplacement) * getTask()->getAction().getLinearSpeed()*MOTOR_CALLBACK;
		deltaPose=b2Transform(b2Vec2(xdistance,
					ydistance), 
					b2Rot(angularDisplacement));
	}
	else{
		deltaPose=*_deltaPose;
	}
	printf("displacement: ");
	debug::print_pose(deltaPose);
	printf("currentVertex = %i, direction =%i\n", currentVertex, currentTask.direction);
	math::applyAffineTrans(deltaPose, g);
	math::applyAffineTrans(-deltaPose, &controlGoal);
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
 