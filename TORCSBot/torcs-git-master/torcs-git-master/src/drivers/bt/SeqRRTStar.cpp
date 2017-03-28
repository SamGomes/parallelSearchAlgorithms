#include "SeqRRTStar.h"


SeqRRTStar::SeqRRTStar(State initialState, int nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments, double actionSimDeltaTime){

	this->maxCost = -1 * DBL_MAX; //force a change
	this->bestState = State();

	this->forwardSegments = forwardSegments;

	int startSegIndex = (currentSearchSeg.id);
	int finalIndex = (startSegIndex + forwardSegments) % (nTrackSegs - 1);

	this->startSegIndex = startSegIndex;
	this->finalIndex = finalIndex;

	this->nIterations = nIterations;
	this->initialState = initialState;

	initGraph();
	pushBackToGraph(this->initialState);

	std::srand(clock());
	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->car = car;

	this->actionSimDeltaTime = actionSimDeltaTime;

}
SeqRRTStar::~SeqRRTStar(){

	//as we do not need the graph anymore we just delete it! (commented because it is used on driver.cpp  for debug purposes)
	deleteGraph();
}

void SeqRRTStar::initGraph(){
	graphSize = (unsigned int)nIterations + 1;
	graph = new State[graphSize]; //upper bound removes useless resizes
	graphIterator = 0;
}

void SeqRRTStar::resizeGraph(unsigned int newSize){
	State* newGraph = new State[newSize];
	graphSize = newSize;

	for (int i = 0; i < graphIterator; i++){
		newGraph[i] = graph[i];
	}
	delete[] graph;
	graph = newGraph;
}
void SeqRRTStar::deleteGraph(){
	delete[] graph;
}


void SeqRRTStar::pushBackToGraph(State &element){
	if (graphIterator == graphSize){
		//graph limited exceeded, resize the graph
		resizeGraph(graphSize + (unsigned int)nIterations);
	}
	element.setMyGraphIndex(graphIterator); //sets the element index
	graph[graphIterator] = element;
	graphIterator++;
}


/*2nd approach (uncomment to activate)*/ 
//the nearest point is the one in which its finalPos prediction ajusts to the current pos and random speed
State SeqRRTStar::nearestNeighbor(State state, State* graph){

	State closestState;
	double minDist = DBL_MAX;
	for (int i = 0; i < graphIterator; i++){

		double dist = UtilityMethods::getPolarQuadranceBetween(state.getVelocity(), graph[i].getVelocity());
		if (dist <= minDist){
			minDist = dist;
			closestState = graph[i];
		}

	}



	return closestState;

}


void SeqRRTStar::generateStates(double nIterations){

	State xRand;


	double maxAa = 5;
	double maxAl = 5;

	double maxDeltaAngle = maxAa*actionSimDeltaTime;
	double maxDeltaIntensity = maxAl*actionSimDeltaTime;

	if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &initialState)){
		return;
	}

	for (int k = 0; k < nIterations; k++){

		//--------------------------------------- generate random sample -----------------------------------------------

		xRand = RandomStateGenerators::uniformRandomState(trackSegArray, nTrackSegs, startSegIndex, finalIndex);
		//xRand = RandomStateGenerators::gaussianRandomState(trackSegArray, nTrackSegs, startSegIndex, finalIndex, initialState.getVelocity());

		//---------------------------------------- select neighbor ----------------------------------------------------

		State xNearest = nearestNeighbor(xRand, graph);
		xRand.setParentGraphIndex(xNearest.getMyGraphIndex());
		xRand.setLevelFromStart(xNearest.getLevelFromStart() + 1);

		//----------------------------------------- constraint checking ------------------------------------------------

		if (abs(xRand.getVelocity().angle- xNearest.getVelocity().angle) > maxDeltaAngle || abs(xRand.getVelocity().intensity-xNearest.getVelocity().intensity) > maxDeltaIntensity*maxDeltaIntensity){
			continue;
		}

		//the delta application also checks if the trajectory is valid
		if (!DeltaFunctions::applyDelta(xRand, xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
			continue;
		}

		//---------------------------------------- calculate best path --------------------------------------------------
	
		//the best state is the one that is furthest from the start lane
		tTrkLocPos xRandLocalPos;
		UtilityMethods::SimpleRtTrackGlobal2Local(xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0);
		xRand.setLocalPos(xRandLocalPos);
		double distFromStart = UtilityMethods::getTrackCenterDistanceBetween(trackSegArray, nTrackSegs, &xRand, &initialState, 500) / xRand.getLevelFromStart();
		xRand.distFromStart = distFromStart;
		pushBackToGraph(xRand);		

		if (distFromStart > maxCost){
			maxCost = distFromStart;
			bestState = xRand;
		}

	}
	

}

/**/


/*1st approach (uncomment to activate)* /
//the nearest point is the one in which its finalPos prediction ajusts to the current pos
State SeqRRTStar::nearestNeighbor(State state, State* graph){

	State closestState = initialState;
	double minDist = DBL_MAX;
	for (int j = 0; j < graphIterator; j++){
		State i = graph[j];
		double dist = UtilityMethods::getEuclideanQuadranceBetween(state.getPos(), i.getPos());
		if (dist < minDist){
			minDist = dist;
			closestState = i;
		}
	}


	return closestState;

}


void SeqRRTStar::generateStates(double nIterations){

	State xRand;

	double velAngleBias = acos(initialState.getVelocity().x / initialState.getVelocity().x);
	double velIntensityBias = sqrt(initialState.getVelocity().x*initialState.getVelocity().x + initialState.getVelocity().y*initialState.getVelocity().y);

	for (int k = 0; k < nIterations; k++){

		xRand = RandomStateGenerators::uniformRandomState(trackSegArray, nTrackSegs, startSegIndex, finalIndex);
		//xRand = RandomStateGenerators::gaussianRandomState(trackSegArray, nTrackSegs, startSegIndex, finalIndex, velAngleBias, velIntensityBias);

		//the generation didnt work
		if (!ConstraintChecking::validPoint(trackSegArray, nTrackSegs, &xRand)){
			continue;
		}


		State xNearest = nearestNeighbor(xRand, graph);
		xRand.setParentGraphIndex(xNearest.getMyGraphIndex());

		//--------------------------------------------------------------------------------------------------------------------------------

		//the normalization didnt work
		if (!DeltaFunctions::applyDelta(xRand, xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
			continue;
		}

		tTrkLocPos xRandLocalPos;
		UtilityMethods::SimpleRtTrackGlobal2Local(xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0);
		xRand.setLocalPos(xRandLocalPos);
		double distFromStart = UtilityMethods::SimpleGetDistanceFromStart(xRand.getLocalPos()) / xRand.getLevelFromStart();
		xRand.distFromStart = distFromStart;
		pushBackToGraph(xRand);

		if (distFromStart > maxCost){
			maxCost = distFromStart;
			bestState = xRand;
		}

	}


}
/**/

State SeqRRTStar::generateRRT(){

	this->generateStates(nIterations);

	//if no path can be found just do emergency cycles! ;)
	if (bestState.getMyGraphIndex() == -1){
		State initialStateCopy = State(initialState);
		initialStateCopy.setInitialState(false);
		initialStateCopy.setParentGraphIndex(initialState.getMyGraphIndex());
		bestState = initialStateCopy;

		pushBackToGraph(initialStateCopy);
	}

	return bestState;
}

//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

std::vector<State*> SeqRRTStar::search(){

	std::vector<State*> path;
	State bestState = this->generateRRT(); //local pointer

	//lets put a path copy in the Heap (calling new), separating the path from the graph eliminated after!
	while (!bestState.getInitialState()){
		path.push_back(new State(bestState));
		bestState = graph[bestState.getParentGraphIndex()];
	}

	return path;
}

std::vector<State> SeqRRTStar::getGraph(){
	std::vector<State>  graphVector = std::vector<State>(graph, &graph[graphIterator - 1]);
	return graphVector;
}


char* SeqRRTStar::getSearchName(){
	return "SequentialRRT";
}