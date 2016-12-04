#include "SeqRRTStar.h"


static bool compareStates(State* s1, State* s2) {
	double s1D = s1->getCost();
	double s2D = s2->getCost();

	return (s1D < s2D);
}


SeqRRTStar::SeqRRTStar(State initialState, double nIterations, tTrack track, tTrackSeg currentSearchSeg, double forwardSegments){
	this->currentSearchSeg = currentSearchSeg;

	while (forwardSegments > 0){
		currentSearchSeg = *currentSearchSeg.next;
		forwardSegments--;
	}
	this->forwardSearchSeg = currentSearchSeg;

	graph.reserve((unsigned int) nIterations+1);
	this->nIterations = nIterations;
	this->initialState = initialState;
	graph.push_back(&this->initialState);
	std::srand(time(NULL));
	this->track = track;
}


SeqRRTStar::~SeqRRTStar(){
	
		
}

//OOOOOOOOOOOOOOOOOOOOOLLLLLLLLLLLLLLLDDDDDDDDDDDDDDDDDD
//// random state is biased, aiming for the state (0,0)
//State* SeqRRTStar::randomState(){
//
//	double biasPos = 1.0f;
//	double biasSteer = 0.0f;
//
//	double influence = 1.0f;
//	double mix = ((double) std::rand() / (double)RAND_MAX)*influence;
//
//
//	double randPos = 2 * ((double)std::rand() / (double)RAND_MAX) - 1;
//	double randSteer = 0.2 * (((PI * std::rand()) / (double)RAND_MAX) - PI / 2);
//	
//	double biasedRandPos = randPos*(1 - mix) + biasPos*mix;
//	double biasedRandSteer = randPos*(1 - mix) + biasSteer*mix;
//
//	return new State(biasedRandPos, biasedRandSteer);
//}


State* SeqRRTStar::randomState(tTrackSeg* initialSeg, tTrackSeg* finalSeg){


	double minXVertex = initialSeg->vertex[0].x;
	double maxXVertex = finalSeg->vertex[0].x;

	double minYVertex = initialSeg->vertex[0].y;
	double maxYVertex = finalSeg->vertex[0].y;

	for (int i = 1; i < 4; i++){
		if (initialSeg->vertex[i].x < minXVertex){
			minXVertex = initialSeg->vertex[i].x;
		}
		if (initialSeg->vertex[i].y < minYVertex){
			minYVertex = initialSeg->vertex[i].y;
		}

		if (finalSeg->vertex[i].x > maxXVertex){
			maxXVertex = finalSeg->vertex[i].x;
		}

		if (finalSeg->vertex[i].y > maxYVertex){
			maxYVertex = finalSeg->vertex[i].y;
		}

	}



	double trackMapXMin = minXVertex;
	double trackMapXMax = maxXVertex;

	double trackMapXDelta = trackMapXMax - trackMapXMin;

	double trackMapYMin = minYVertex;
	double trackMapYMax = maxYVertex;

	double trackMapYDelta = trackMapYMax - trackMapYMin;


	double trackMapZMin = 0;
	double trackMapZMax = 20;

	double trackMapZDelta = trackMapZMax - trackMapZMin;


	double minSpeed = 30;
	double maxSpeed = 70;

	double speedDelta = maxSpeed - minSpeed;


	double minAccel = 0;
	double maxAccel = 10;

	double accelDelta = maxAccel - minAccel;


	double randPosX = trackMapXDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapXMin;
	double randPosY = trackMapYDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapYMin;
	double randPosZ = trackMapZDelta * ((double)std::rand() / (double)RAND_MAX) + trackMapZMin;

	tPosd randPos;
	randPos.x = randPosX;
	randPos.y = randPosY;
	randPos.z = randPosZ;

	double randSpeedX = speedDelta * ((double)std::rand() / (double)RAND_MAX) + minSpeed;
	double randSpeedY = speedDelta * ((double)std::rand() / (double)RAND_MAX) + minSpeed;
	double randSpeedZ = speedDelta * ((double)std::rand() / (double)RAND_MAX) + minSpeed;

	tPosd randSpeed;
	randSpeed.x = randSpeedX;
	randSpeed.y = randSpeedY;
	randSpeed.z = randSpeedZ;

	double randAccelX = accelDelta * ((double)std::rand() / (double)RAND_MAX) + minAccel;
	double randAccelY = accelDelta * ((double)std::rand() / (double)RAND_MAX) + minAccel;
	double randAccelZ = accelDelta * ((double)std::rand() / (double)RAND_MAX) + minAccel;

	tPosd randAccel;
	randAccel.x = randAccelX;
	randAccel.y = randAccelY;
	randAccel.z = randAccelZ;

	return new State(randPos, randSpeed, randAccel);
}

void SeqRRTStar::normalizeState(State* state, State* parent){

	double diffPathCost = state->getPathCost() - parent->getPathCost();

	if (diffPathCost<0 && parent->getParent()!=NULL){
		state->setParent(parent->getParent());
		parent->setParent(state);
	}
}


double  SeqRRTStar::evaluateCost(State* state){
	tPosd finalPos = state->getPos();
	tPosd a = state->getAcceleration();
	tPosd v0 = state->getSpeed();
	double t = this->actionSimDeltaTime;

	finalPos.x += 0.5*a.x*t*t + v0.x;
	finalPos.y += 0.5*a.y*t*t + v0.y;
	finalPos.z += 0.5*a.z*t*t + v0.z;

	return finalPos.x*finalPos.x + finalPos.y*finalPos.y;
}

double  SeqRRTStar::evaluatePathCost(State* s1,State* s2){
	return localDistance(s1,s2,20);
}


//the nearest point is the one in which its finalPos ajusts to the points current pos
State* SeqRRTStar::nearestNeighbor(State* state, std::vector<State*> graph){
	std::sort(graph.begin(), graph.end(), compareStates);

	tPosd currentPos = state->getPos();

	double currPosMod = currentPos.x*currentPos.x + currentPos.y*currentPos.y;

	if (graph.size() == 1){
		return graph[0];
	}

	for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i){
		double currCost = (*i)->getCost();
		if (currCost > currPosMod && i != graph.begin()){
			double lastCost = (*(i-1))->getCost();
			if (abs(currCost - currPosMod) < abs(lastCost - currPosMod)){
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

//In this version there are two types of cost:
// - the cost of the node represents the smoothness between the transition from the node to its son
// - the path cost is the distance travelled by the path along the track

State* SeqRRTStar::generateRRT(){
	double maxPathCost = -1*DBL_MAX; //force a change
	State* bestState = NULL;

	for (int k = 0; k < nIterations; k++){

		State* xRand = randomState(&currentSearchSeg, &forwardSearchSeg);

		while (!validPoint(xRand))	{
			xRand = randomState(&currentSearchSeg, &forwardSearchSeg);
		}

		double xRandNearCost = evaluateCost(xRand);
		xRand->setCost(xRandNearCost);

		State* xNearest = nearestNeighbor(xRand, graph); //after cost evaluation
		xRand->setParent(xNearest);

		double cMin = xNearest->getPathCost() + evaluatePathCost(xNearest, xRand);
		xRand->setPathCost(cMin);

		normalizeState(xRand,xNearest);

		cMin = xRand->getParent()->getPathCost() + evaluatePathCost(xNearest, xRand);
		xRand->setPathCost(cMin);

		/*std::vector<State*> nearNeighbors = nearestNeighbors(xRand, graph);
		State* xMin = xNearest;
*/

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


		if (xRand->getPathCost() >= maxPathCost){
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

	path.reserve((unsigned int) nIterations + 1); //upper bound on path size (if not reserved creates bottleneck bellow)
	
	while (currState.getParent() != NULL){
		path.push_back(currState);
		currState = *(currState.getParent());
	}

	for (int i = 1; i <= nIterations; i++){
		delete graph[i];
	}

	//path.push_back(currState); // pushes initial node
	return path;

}


bool SeqRRTStar::validPoint(State* targetState){
	////point on oponent?

	//if (target.x <= (opponent->getCarPtr()->_pos_X + 20) &&
	//	target.x >= (opponent->getCarPtr()->_pos_X - 20) &&
	//	target.y <= (opponent->getCarPtr()->_pos_Y + 20) &&
	//	target.y >= (opponent->getCarPtr()->_pos_Y - 20)){
	//	return false;

	//}



	tPosd target = targetState->getPos();
	
	//point outside track?

	tTrkLocPos targetLocalPos;
	tTrackSeg* seg = track.seg;
	tTrackSeg* currSeg = seg->next;

	
	while (currSeg != seg ){
		RtTrackGlobal2Local(currSeg, target.x, target.y, &targetLocalPos, TR_LPOS_MAIN);
		if (targetLocalPos.toRight > 0 && targetLocalPos.toLeft > 0){
			targetState->setPosSeg(*currSeg);
			return true;
		}
		currSeg = currSeg->next;
	}
	return false;
}

double SeqRRTStar::localDistance(State* s1, State* s2, int fwdLimit){

	tTrkLocPos l1,l2;

	RtTrackGlobal2Local(&(s1->getPosSeg()), s1->getPos().x, s1->getPos().y, &l1, TR_LPOS_MAIN);
	RtTrackGlobal2Local(&(s2->getPosSeg()), s2->getPos().x, s2->getPos().y, &l2, TR_LPOS_MAIN);

	tTrackSeg	*l1Seg = l1.seg;
	tTrackSeg	*l2Seg = l2.seg;


	if (l1Seg->id == l2Seg->id){
		double distance = l2.toStart - l1.toStart;
		return distance;
	}
	else{
		tTrackSeg* currSegFwd = l1Seg->next;
		tTrackSeg* currSegBwd = l1Seg->prev;
		double bwdDist = 0;
		double fwdDist = 0;
		while (currSegFwd != currSegBwd && fwdLimit>0){
			if (currSegFwd->id == l2Seg->id){
				return fwdDist+(l1Seg->length - l1.toStart);
			}
			if (currSegBwd->id == l2Seg->id){
				return -1*(bwdDist + l1.toStart);
			}
			bwdDist += currSegBwd->length;
			fwdDist += currSegFwd->length;
			currSegFwd = currSegFwd->next;
			currSegBwd = currSegBwd->prev;
			fwdLimit--;
		}
		return -1 * DBL_MAX; //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
	}
}