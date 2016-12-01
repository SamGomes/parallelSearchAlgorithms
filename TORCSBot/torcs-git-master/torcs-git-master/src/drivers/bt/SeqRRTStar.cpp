#include "SeqRRTStar.h"


static bool compareStates(State* s1, State* s2) {
	double s1D = s1->getCost();
	double s2D = s2->getCost();

	return (s1D > s2D);
}


SeqRRTStar::SeqRRTStar(State* initialState, double nIterations, tTrack track){
	graph.reserve((unsigned int) nIterations+1);
	this->nIterations = nIterations;
	graph.push_back(initialState);
	std::srand(time(NULL));
	this->track = track;
}


SeqRRTStar::~SeqRRTStar(){
	for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i)
		delete (*i);
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


State* SeqRRTStar::randomState(){

	double trackMapXMin = 0;
	double trackMapXMax = 800;

	double trackMapXDelta = trackMapXMax - trackMapXMin;

	double trackMapYMin = 0;
	double trackMapYMax = 500;

	double trackMapYDelta = trackMapYMax - trackMapYMin;


	double trackMapZMin = 0;
	double trackMapZMax = 20;

	double trackMapZDelta = trackMapZMax - trackMapZMin;


	double minSpeed = 0;
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

void SeqRRTStar::normalizeState(State* state){
	tPosd normalizedPos;
		
	normalizedPos.x = normalizeDiff(state, state->getPos().x, state->getParent()->getPos().x, NEIGHBOR_DELTA_POS);
	normalizedPos.y = normalizeDiff(state, state->getPos().y, state->getParent()->getPos().y, NEIGHBOR_DELTA_POS);
	normalizedPos.z = normalizeDiff(state, state->getPos().z, state->getParent()->getPos().z, NEIGHBOR_DELTA_POS);
								
	tPosd normalizedSpeed;
	
	normalizedSpeed.x = normalizeDiff(state, state->getSpeed().x, state->getParent()->getSpeed().x, NEIGHBOR_DELTA_SPEED);
	normalizedSpeed.y = normalizeDiff(state, state->getSpeed().y, state->getParent()->getSpeed().y, NEIGHBOR_DELTA_SPEED);
	normalizedSpeed.z = normalizeDiff(state, state->getSpeed().z, state->getParent()->getSpeed().z, NEIGHBOR_DELTA_SPEED);

	state->setCommands(normalizedPos, normalizedSpeed, state->getAcceleration());

}

double SeqRRTStar::normalizeDiff(State* state, double num1, double num2, double delta){
	
	double difference = num1 - num2;

	if (abs(difference) <= delta){
		return num1;
	}
	else{
		if (difference > 0){
			num1 = num1 - (difference - delta);
			return num1;
		}
		else{
			num1 = num1 + (-difference - delta);
			return num1;
		}
	}

}


double  SeqRRTStar::evaluatePathCost(State* s1, State* s2){
	return computeCost(s1->getPos(), s2->getPos(), s1->getSpeed(), s2->getSpeed(), s1->getAcceleration(), this->actionSimDeltaTime);
}



double SeqRRTStar::computeSmoothness(tPosd initialPos, tPosd finalPos, tPosd initialSpeed, tPosd finalSpeed, tPosd acceleration, double simDeltaTime){

	double d = finalPos.x*finalPos.x + finalPos.y*finalPos.y;
	double d0 = initialPos.x*initialPos.x + initialPos.y*initialPos.y;

	double a = acceleration.x*acceleration.y;
	double v0 = initialSpeed.x*initialSpeed.y;

	return abs((d - d0) / ((0.5*a*a*simDeltaTime) + v0*simDeltaTime));

}

double SeqRRTStar::computeCost(tPosd initialPos, tPosd finalPos, tPosd initialSpeed, tPosd finalSpeed, tPosd acceleration, double simDeltaTime){

	double smoothness = this->computeSmoothness(initialPos, finalPos, initialSpeed, finalSpeed, acceleration, simDeltaTime);

	double x = finalPos.x - initialPos.x;
	double y = finalPos.y - initialPos.y;

	double distance = (x*x + y*y);

	//this->cost = 0.2*smoothness + 0.8*distance;
	return distance;
}



State* SeqRRTStar::nearestNeighbor(State* state, std::vector<State*> graph){
	std::sort(graph.begin(), graph.end(), compareStates);

	/*if (graph.size() < 2){
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
	}*/


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

		if (!validPoint(xRand))
			continue;

		double xRandNearPathCost = evaluatePathCost(xNearest, xRand);
		double cMin = xNearest->getPathCost() + xRandNearPathCost;

		xRand->setCost(xRandNearPathCost);
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

	path.reserve((unsigned int) nIterations + 1); //upper bound on path size (if not reserved creates bottleneck bellow)
	
	while (currState.getParent() != NULL){
		path.push_back(currState);
		currState = *(currState.getParent());
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
	tPosd parentPos = targetState->getParent()->getPos();
	
	//point outside track?

	tTrkLocPos targetLocalPos;
	tTrkLocPos parentLocalPos;
	tTrackSeg* seg = track.seg;
	tTrackSeg* currSeg = seg->next;

	
	while (currSeg != seg ){
		RtTrackGlobal2Local(currSeg, target.x, target.y, &targetLocalPos, TR_LPOS_MAIN);
		RtTrackGlobal2Local(currSeg, parentPos.x, parentPos.y, &parentLocalPos, TR_LPOS_MAIN);

		if (targetLocalPos.toRight > -5 && targetLocalPos.toLeft > -5){
			
			if (mostFar(targetLocalPos, parentLocalPos,5)){
				return true;
			}else{
				return false;
			}
				
		}
		currSeg = currSeg->next;
	}
	return false;
}

bool SeqRRTStar::mostFar(tTrkLocPos l1, tTrkLocPos l2,int fwdLimit){
	tTrackSeg	*l1Seg = l1.seg;
	tTrackSeg	*l2Seg = l2.seg;


	if (l1Seg == l2Seg){
		double distance = l2.toStart - l1.toStart;
		if (distance > 0) return false;
		else return true;
	}
	else{
		tTrackSeg* currSegFwd = l1Seg->next;
		tTrackSeg* currSegBwd = l1Seg->prev;

		while (currSegFwd != currSegBwd && fwdLimit>0){
			if (currSegFwd == l2Seg){
				return false;
			}
			if (currSegBwd == l2Seg){
				return true;
			}

			currSegFwd = currSegFwd->next;
			currSegBwd = currSegBwd->prev;
			fwdLimit--;
		}
		return false; //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
	}
}