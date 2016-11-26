#include "SeqRRTStar.h"


static bool compareStates(State* s1, State* s2) {
	double s1D = s1->getCost();
	double s2D = s2->getCost();
	return (s1D > s2D);
}


SeqRRTStar::SeqRRTStar(State* initialState, double nIterations,tTrack track){
	graph.reserve(nIterations+1);
	this->nIterations = nIterations;
	graph.push_back(initialState);
	std::srand(time(NULL));
	this->track = track;
}


SeqRRTStar::~SeqRRTStar(){
	for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i)
		delete (*i);
}

// random state is biased, aiming for the state (0,0)
State* SeqRRTStar::randomState(){

	double biasPos = 1.0f;
	double biasSteer = 0.0f;

	double influence = 1.0f;
	double mix = ((double) std::rand() / (double)RAND_MAX)*influence;


	double randPos = 2 * ((double)std::rand() / (double)RAND_MAX) - 1;
	double randSteer = 0.2 * (((PI * std::rand()) / (double)RAND_MAX) - PI / 2);
	
	double biasedRandPos = randPos*(1 - mix) + biasPos*mix;
	double biasedRandSteer = randPos*(1 - mix) + biasSteer*mix;

	return new State(biasedRandPos, biasedRandSteer);
}


void SeqRRTStar::normalizeState(State* state){
	state->setCommands(normalizeDiff(state->getPedalPos(), state->getParent()->getPedalPos(), NEIGHBOR_DELTA_PEDALPOS),
					   normalizeDiff(state->getSteerAngle(), state->getParent()->getSteerAngle(), NEIGHBOR_DELTA_STEERANGLE));
}

double SeqRRTStar::normalizeDiff(double num1, double num2, double delta){
	
	double difference = num1 - num2;
	if (difference <= delta){
		return difference;
	}
	else{
		if (difference > 0){
			return num1-(difference - delta);
		}
		else{
			return num1+(difference - delta);
		}
	}

}


double  SeqRRTStar::evaluatePathCost(State* s1, State* s2){
	//Here is where the cost of a state (an action) is calculated

	//return (s2->getFinalPos() - s1->getInitialPos()) / (0.5*s1->getAcceleration()*s1->getAcceleration() + s1->get)


	//MOCK!
	//return std::abs(std::rand() % 10);

	return s2->getCost();

}

State* SeqRRTStar::nearestNeighbor(State* state, std::vector<State*> graph){
	std::sort(graph.begin(), graph.end(), compareStates);

	if (graph.size() < 2){
		return graph[0];
	}

	for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i){
		double currCost = (*i)->getCost();
		double stateCost = state->getCost();
		if (currCost > stateCost && i!=graph.begin()){
			double lastCost = (*(i-1))->getCost();
			if (abs(currCost - stateCost) < abs(lastCost - stateCost)){
				return (*i);
			}
			else{
				return(*(i - 1));
			}
		}
	}

	return graph.back();

}

std::vector<State*> SeqRRTStar::nearestNeighbors(State* state, std::vector<State*> graph){
	std::vector<State*> neighbors = std::vector<State*>();

	std::vector<State*> auxGraph = graph;

	int effectiveIterations = (auxGraph.size() -1 < NEIGHBOR_SAMPLE_BOUNDARY) ? (auxGraph.size() - 1) : NEIGHBOR_SAMPLE_BOUNDARY;

	for (int i = 0; i < effectiveIterations; i++){

		int index = std::rand() % auxGraph.size();
		//remove initial node
		if (state->getParent()==NULL){
			auxGraph.erase(std::remove(auxGraph.begin(), auxGraph.end(), state), auxGraph.end());
		}
		State* currNearestNeighbor = nearestNeighbor(state, auxGraph);
		neighbors.push_back(currNearestNeighbor);
		auxGraph.erase(std::remove(auxGraph.begin(), auxGraph.end(), currNearestNeighbor), auxGraph.end());
	}
	return neighbors;
}

bool SeqRRTStar::considerFinalState(State* finalState){
	//NOT SURE IF NEEDED
	return true;
}

State* SeqRRTStar::generateRRT(){
	double maxPathCost = -1; //force a change
	State* bestState = NULL;

	for (int k = 0; k < nIterations; k++){

		State* xRand = randomState();
		
		State* xNearest = nearestNeighbor(xRand, graph);

		xRand->setParent(xNearest);
		normalizeState(xRand);

		double xRandNearPathCost = evaluatePathCost(xNearest, xRand);
		double cMin = xNearest->getPathCost() + xRandNearPathCost;

		xRand->setPathCost(cMin);


		std::vector<State*> nearNeighbors = nearestNeighbors(xRand, graph);

		State* xMin = xNearest;


		////check if there is a better path
		//for (State* xCurr : nearNeighbors){
		//	double cCurr = xCurr->getPathCost() + evaluatePathCost(xCurr, xRand);
		//	if (cCurr > cMin){
		//		xMin = xCurr;
		//		cMin = cCurr;
		//	}
		//}

		//xRand->setParent(xMin);
		//xRand->setPathCost(cMin);



		
		////reorder path to create a better path (it is not working now as it creates loops)
		//for (State* xCurr : nearNeighbors){
		//	//except xMin
		//	if (xCurr == xMin || xRand->getParent() == NULL || xRand->getParent() != xCurr) 
		//		continue;

		//	double cCurr = xRand->getDistance() + evaluateDistance(xCurr, xRand);
		//	if (cCurr > xCurr->getDistance()){
		//		xCurr->setParent(xRand);
		//		xCurr->setDistance(cCurr);
		//	}

		//	//missing costs update to the path!

		//}


		/*if (!validPoint(xRand->getFinalPos()))
			continue;
*/

		if (xRand->getPathCost() > maxPathCost){
			maxPathCost = xRand->getPathCost();
			bestState = xRand;
		}
		graph.push_back(xRand);

	}
	if (bestState == NULL){
		return generateRRT();
	}
	return bestState;
}


std::vector<State> SeqRRTStar::search(){
	std::vector<State> path;
	State* bestState = this->generateRRT();
	State currState = *bestState;

	path.reserve(nIterations + 1); //upper bound on path size (if not reserved creates bottleneck bellow)
	
	while (currState.getParent() != NULL){
		path.push_back(currState);
		currState = *(currState.getParent());
	}

	return path;

}


bool SeqRRTStar::validPoint(tPosd target){
	////point on oponent?

	//if (target.x <= (opponent->getCarPtr()->_pos_X + 20) &&
	//	target.x >= (opponent->getCarPtr()->_pos_X - 20) &&
	//	target.y <= (opponent->getCarPtr()->_pos_Y + 20) &&
	//	target.y >= (opponent->getCarPtr()->_pos_Y - 20)){
	//	return false;

	//}

	//point outside track?
	tTrkLocPos pos;
	tTrackSeg* seg = track.seg;
	tTrackSeg* currSeg = seg->next;
	while (currSeg != seg){
		RtTrackGlobal2Local(currSeg, target.x, target.y, &pos, TR_LPOS_MAIN);
		if (pos.toRight > 0 && pos.toLeft > 0){
			return true;
		}

		currSeg = currSeg->next;
	}
	return false;
}

