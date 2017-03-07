#include "SeqRRTStar.h"


SeqRRTStar::SeqRRTStar(State initialState, double nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments){

	this->maxPathCost = -1 * DBL_MAX; //force a change
	this->bestState = nullptr;

	this->forwardSegments = forwardSegments;

	int startSegIndex = (currentSearchSeg.id);
	int finalIndex = (startSegIndex + forwardSegments) % (nTrackSegs - 1);

	this->currentSearchSeg = trackSegArray[startSegIndex];
	this->forwardSearchSeg = trackSegArray[finalIndex];

	this->nIterations = nIterations;
	this->initialState = new State(initialState);

	initGraph();
	pushBackToGraph(this->initialState);

	std::srand(time(NULL));
	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->car = car;

}
SeqRRTStar::~SeqRRTStar(){

	//as we do not need the graph anymore we just delete it! (commented because it is used on driver.cpp  for debug purposes)
	deleteGraph();
}

void SeqRRTStar::initGraph(){
	graphSize = (unsigned int)nIterations + 1;
	graph = new State*[graphSize]; //upper bound removes useless resizes
	graphIterator = 0;
}

void SeqRRTStar::resizeGraph(unsigned int newSize){
	State** newGraph = new State*[newSize];
	graphSize = newSize;

	for (int i = 0; i < graphIterator; i++){
		newGraph[i] = graph[i];
	}
	delete[] graph;
	graph = newGraph;
}
void SeqRRTStar::deleteGraph(){
	for (int i = 0; i < graphIterator; i++){
		delete graph[i];
	}

	delete[] graph;
}


void SeqRRTStar::pushBackToGraph(State* element){
	if (graphIterator == graphSize){
		//graph limited exceeded, resize the graph
		resizeGraph(graphSize + (unsigned int)nIterations);
	}
	graph[graphIterator] = element;
	graphIterator++;
}

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

	double minSpeed = 0;
	double maxSpeed = 60;

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

//the nearest point is the one in which its finalPos prediction ajusts to the current pos
State* SeqRRTStar::nearestNeighbor(State* state, State** graph){

	State* closestState = initialState;
	double minCost = DBL_MAX;
	for (int j = 0; j < graphIterator; j++){
		State* i = graph[j];
		double currCost = EvalFunctions::evaluateStateCost(i, state, this->actionSimDeltaTime);
		if (minCost > currCost){
			minCost = currCost;
			closestState = i;
		}
	}


	return closestState;

}


void SeqRRTStar::generateStates(double nIterations){

	State* xRand = nullptr;
	

	for (int k = 0; k < nIterations; k++){

		xRand = randomState(&currentSearchSeg, &forwardSearchSeg);

		//the generation didnt work
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, xRand, 0)){
			delete xRand;
			continue;
		}


		State* xNearest = nearestNeighbor(xRand, graph);
		xRand->setParent(xNearest);

		//--------------------------------------------------------------------------------------------------------------------------------

		DeltaHeuristics::applyDelta(xRand, xNearest, trackSegArray, nTrackSegs, forwardSegments, NEIGHBOR_DELTA_POS, NEIGHBOR_DELTA_SPEED);

		double cMin = xNearest->getPathCost() + EvalFunctions::evaluatePathCost(trackSegArray, nTrackSegs, xNearest, xRand, this->forwardSegments); //redifine path cost for new coords
		//double cMin = xNearest->getPathCost() + EvalFunctions::oldEvaluatePathCost( xNearest, xRand, this->forwardSegments);
		xRand->setPathCost(cMin);


		//the normalization didnt work
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, xRand, -1)){
			delete xRand;
			continue;
		}


		if (xRand->getPathCost() >= maxPathCost){
			maxPathCost = xRand->getPathCost();
			bestState = xRand;
		}
		pushBackToGraph(xRand);

	}
	

}

State* SeqRRTStar::generateRRT(){

	this->generateStates(nIterations);

	//if no path can be found just do emergency cycles! ;)
	if (bestState == nullptr){

		this->generateStates(this->numberOfEmergencyCycles);

		if (bestState == nullptr){
			State* initialStateCopy = new State(*initialState);
			initialStateCopy->setInitialState(false);
			initialStateCopy->setParent(initialState);
			bestState = initialStateCopy;

			pushBackToGraph(initialStateCopy);
		}

	}

	return bestState;
}

//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

void SeqRRTStar::updateCar(tCarElt car){
	this->car = car;
}
std::vector<State*> SeqRRTStar::search(){

	std::vector<State*> path;

	State* bestState = this->generateRRT(); //local pointer


	//lets put a path copy in the Heap (calling new), separating the path from the graph eliminated after!
	while (!bestState->getInitialState()){
		path.push_back(new State(*bestState));
		bestState = bestState->getParent();
	}

	return path;
}
std::vector<State*> SeqRRTStar::getGraph(){
	std::vector<State*>  graphVector = std::vector<State*>(graph, &graph[graphIterator - 1]);

	return graphVector;
}