#include "SeqRRTOpt.h"


static bool compareStates(State* s1, State* s2) {
	double s1D = s1->getPathCost();
	double s2D = s2->getPathCost();

	return (s1D < s2D);
}


SeqRRTOpt::SeqRRTOpt(State initialState, double nIterations, tCarElt car, tTrack track, tTrackSeg currentSearchSeg, int forwardSegments){

	this->deleteBestState = false;

	this->currentSearchSeg = currentSearchSeg;

	this->forwardSegments = forwardSegments;

	while (forwardSegments > 0){
		currentSearchSeg = *currentSearchSeg.next;
		forwardSegments--;
	}
	this->forwardSearchSeg = currentSearchSeg;
	this->nIterations = nIterations;
	this->initialState = new State(initialState);
	graph.reserve((unsigned int)nIterations + 1); //to increase performance (pushes can now have constant time)
	graph.push_back(this->initialState);
	std::srand(time(NULL));
	this->track = track;
	this->car = car;


	this->generateRRT();


}


SeqRRTOpt::~SeqRRTOpt(){
	if (deleteBestState){
		delete bestState;
	}

	//as we do not need the graph anymore we just delete it! (commented because it is used on driver.cpp  for debug purposes)
	/*for (int i = 0; i < graph.size(); i++){
	delete graph[i];
	}*/
}


State* SeqRRTOpt::randomState(){


	/*double minXVertex = initialSeg->vertex[0].x;
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

	}*/

	double maxXVertex = track.max.x;
	double minXVertex = track.min.x;
	double maxYVertex = track.max.y;
	double minYVertex = track.min.y;

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
	double maxSpeed = 50;
	

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

void SeqRRTOpt::normalizeState(State* state, State* parent){

	double diffX = abs(state->getPos().x - parent->getPos().x);
	double diffY = abs(state->getPos().y - parent->getPos().y);


	double angle = (atan2((state->getPos().y - parent->getPos().y), (state->getPos().x - parent->getPos().x)));

	NORM0_2PI(angle);

	double r = this->NEIGHBOR_DELTA_POS;

	tPosd newPos = tPosd();

	double auxCalc = (sqrt(diffX*diffX + diffY*diffY));



	if (angle>=0 && angle <PI/2){
		newPos = { parent->getPos().x + (r*diffX) / auxCalc, parent->getPos().y + (r*diffY) / auxCalc, state->getPos().z };
		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());
	}

	if (angle >= PI / 2 && angle <PI){
		newPos = { parent->getPos().x - (r*diffX) / auxCalc, parent->getPos().y + (r*diffY) / auxCalc, state->getPos().z };
		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());


	}

	if (angle >= PI && angle < 3*PI / 2){
		newPos = { parent->getPos().x - (r*diffX) / auxCalc, parent->getPos().y - (r*diffY) / auxCalc, state->getPos().z };
		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());
	}

	if (angle >= 3*PI / 2 && angle <2*PI){
		newPos = { parent->getPos().x + (r*diffX) / auxCalc, parent->getPos().y - (r*diffY) / auxCalc, state->getPos().z };
		state->setCommands(newPos, state->getSpeed(), state->getAcceleration());
	}

	
	

}


double  SeqRRTOpt::evaluateCost(State* state){
	tPosd finalPos = state->getPos();
	tPosd a = state->getAcceleration();
	tPosd v0 = state->getSpeed();
	double t = this->actionSimDeltaTime;


	/*finalPos.x += 0.5*a.x*t*t + v0.x;
	finalPos.y += 0.5*a.y*t*t + v0.y;
	finalPos.z += 0.5*a.z*t*t + v0.z;*/

	return sqrt(finalPos.x*finalPos.x + finalPos.y*finalPos.y);
}

double  SeqRRTOpt::evaluatePathCost(State* s1, State* s2){
	return localDistance(s1, s2, this->forwardSegments);
}


//the nearest point is the one in which its finalPos ajusts to the points current pos
State* SeqRRTOpt::nearestNeighbor(State* state, std::vector<State*> graph){
	State* closestState = initialState;
	double minCost = DBL_MAX;
	for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i){
		double a = (*i)->getAcceleration().x*(*i)->getAcceleration().y;
		double v0 = (*i)->getSpeed().x*(*i)->getSpeed().y;
		double currCost = ((((*i)->getPos().x) - (state->getPos().x))*(((*i)->getPos().x) - (state->getPos().x)) + (((*i)->getPos().y) - (state->getPos().y))*(((*i)->getPos().y) - (state->getPos().y))) + a*this->actionSimDeltaTime*this->actionSimDeltaTime + v0*this->actionSimDeltaTime;
		if (minCost > currCost){
			minCost = currCost;
			closestState = *i;
		}
	}


	return closestState;

}


std::vector<State*> SeqRRTOpt::nearestNeighbors(State* state, std::vector<State*> graph){
	std::vector<State*> neighbors = std::vector<State*>();

	std::vector<State*> auxGraph = graph;

	int effectiveIterations = (auxGraph.size() - 1 < NEIGHBOR_SAMPLE_BOUNDARY) ? (auxGraph.size() - 1) : NEIGHBOR_SAMPLE_BOUNDARY;

	for (int i = 0; i < effectiveIterations; i++){
		int index = std::rand() % auxGraph.size();
		//remove initial node
		if (state->getInitialState()){
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

void SeqRRTOpt::generateRRT(){

	for (int k = 0; k < nIterations; k++){

		State* xRand = nullptr;

		delete xRand;
		xRand = randomState();

		double xRandNearCost = evaluateCost(xRand);
		//xRand->setCost(xRandNearCost);

		State* xNearest = nearestNeighbor(xRand, graph); //after cost evaluation
		xRand->setParent(xNearest);

		//--------------------------------------------------------------------------------------------------------------------------------

		normalizeState(xRand, xNearest);

		xRandNearCost = evaluateCost(xRand); //redifine cost for new coords
		//xRand->setCost(xRandNearCost);

		//double cMin = xNearest->getPathCost() + evaluatePathCost(xNearest, xRand); //redifine path cost for new coords
		//xRand->setPathCost(cMin);



		graph.push_back(xRand);

	}



}


std::vector<State*> SeqRRTOpt::search(){

	std::vector<State*> path;

	State* bestState = initialState; //local pointer


	//lets put the path in the Heap (calling new), separating the path from the graph eliminated in the destructor!
	while (!bestState->getInitialState()){
		path.push_back(new State(*bestState));
		bestState = bestState->getParent();
	}


	return path;
}

std::vector<State*> SeqRRTOpt::getGraph(){
	return graph;
}



bool SeqRRTOpt::validPoint(State* targetState, double distFromSides){
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


	while (currSeg != seg){
		RtTrackGlobal2Local(currSeg, target.x, target.y, &targetLocalPos, TR_LPOS_MAIN);
		if (targetLocalPos.toRight >  distFromSides && targetLocalPos.toLeft >  distFromSides){
			targetState->setPosSeg(*currSeg);
			return true;
		}
		currSeg = currSeg->next;
	}
	return false;
}

double SeqRRTOpt::localDistance(State* s1, State* s2, int fwdLimit){

	tTrkLocPos l1, l2;

	RtTrackGlobal2Local(&(s1->getPosSeg()), s1->getPos().x, s1->getPos().y, &l1, TR_LPOS_MAIN);

	RtTrackGlobal2Local(&(s2->getPosSeg()), s2->getPos().x, s2->getPos().y, &l2, TR_LPOS_MAIN);



	tTrackSeg	*l1Seg = l1.seg;
	tTrackSeg	*l2Seg = l2.seg;


	if (l1Seg->id == l2Seg->id){
		double distance = 0;

		switch (l2Seg->type) {
		case TR_STR:
			distance = l2.toStart - l1.toStart;
			break;
		default:
			distance = l2.toStart*l2Seg->radius - l1.toStart*l2Seg->radius;
			break;
		}

		return distance;
	}
	else{
		tTrackSeg* currSegFwd = l1Seg->next;
		tTrackSeg* currSegBwd = l1Seg->prev;
		double bwdDist = 0;
		double fwdDist = 0;
		while (currSegFwd != currSegBwd && fwdLimit>0){

			if (currSegFwd->id == l2Seg->id){


				switch (currSegBwd->type) {
				case TR_STR:
					return fwdDist + (l1Seg->length - l1.toStart);
				default:
					return fwdDist + (l1Seg->length*currSegBwd->radius - l1.toStart*currSegBwd->radius);
				}

			}

			if (currSegBwd->id == l2Seg->id){

				switch (currSegBwd->type) {
				case TR_STR:
					return -1 * (bwdDist + l1.toStart);
				default:
					return -1 * (bwdDist + l1.toStart*currSegBwd->radius);
				}


			}

			switch (currSegBwd->type) {
			case TR_STR:
				bwdDist += currSegBwd->length;
				break;
			default:
				bwdDist += currSegBwd->length * currSegBwd->radius;
				break;
			}

			switch (currSegFwd->type) {
			case TR_STR:
				fwdDist += currSegFwd->length;
				break;
			default:
				fwdDist += currSegFwd->length *currSegFwd->radius;
				break;
			}

			currSegFwd = currSegFwd->next;
			currSegBwd = currSegBwd->prev;
			fwdLimit--;
		}
		return -1 * (bwdDist); //when they exceed forward segs limit (or equidistant if limit exceeds half the segments)
	}
}