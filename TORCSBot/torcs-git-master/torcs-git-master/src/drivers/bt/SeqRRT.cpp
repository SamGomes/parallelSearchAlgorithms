#include "SeqRRT.h"


SeqRRT::SeqRRT(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration){
	this->maxCost = -1 * DBL_MAX; //force a change
	this->bestState = State();

	this->startSegIndex = startSegIndex;
	this->finalIndex = finalIndex;

	this->nIterations = nIterations;
	this->initialState = initialState;

	initGraph();
	pushBackToGraph(this->initialState);

	std::srand(clock());
	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;

	this->actionSimDeltaTime = actionSimDeltaTime;
}
SeqRRT::~SeqRRT(){
	//as we do not need the graph anymore we just delete it! (commented because it is used on driver.cpp  for debug purposes)
	deleteGraph();
}

void SeqRRT::initGraph(){
	graphSize = (unsigned int)nIterations + 1;
	graph = new State[graphSize]; //upper bound removes useless resizes
	graphIterator = 0;
}

void SeqRRT::resizeGraph(unsigned int newSize){ //can be more usefull for partial searches (maybe future work)
	State* newGraph = new State[newSize];
	graphSize = newSize;

	for (int i = 0; i < graphIterator; i++){
		newGraph[i] = graph[i];
	}
	delete[] graph;
	graph = newGraph;
}
void SeqRRT::deleteGraph(){
	delete[] graph;
}


void SeqRRT::pushBackToGraph(State &element){
	if (graphIterator == graphSize){
		//graph limited exceeded, resize the graph
		resizeGraph(graphSize + (unsigned int)nIterations);
	}
	element.setMyGraphIndex(graphIterator); //sets the element index
	graph[graphIterator] = element;
	graphIterator++;
}


void SeqRRT::generateStates(double nIterations){

	State xRand;

	for (int k = 0; k < nIterations; k++){

		//--------------------------------------- generate random sample -----------------------------------------------
		xRand = RandomStateGenerators::uniformRandomState(trackSegArray, nTrackSegs,nullptr);
		//xRand = RandomStateGenerators::gaussianRandomState(trackSegArray, nTrackSegs, initialState.getVelocity());

		//---------------------------------------- select neighbor ----------------------------------------------------
		State xNearest = UtilityMethods::nearestNeighbor(xRand, graph,graphIterator);
		xRand.setParentGraphIndex(xNearest.getMyGraphIndex());
		xRand.setLevelFromStart(xNearest.getLevelFromStart() + 1);

		//----------------------------------------- constraint checking ------------------------------------------------
		//the delta application also checks if the trajectory is valid
		if (!DeltaFunctions::applyDelta(&xRand, &xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
			continue;
		}

		//---------------------------------------- calculate best path --------------------------------------------------
		//the best state is the one that is furthest from the start lane
		tStateRelPos xRandLocalPos;
		UtilityMethods::SimpleRtTrackGlobal2Local(&xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0);
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

State SeqRRT::generateRRT(){
	this->generateStates(nIterations);
	//if no path can be found just return a copy of the initial state! ;)
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

std::vector<State*> SeqRRT::search(){
	std::vector<State*> path;
	State bestState = this->generateRRT();

	//---------------------- BACKTRACKING ----------------------------------
	
	//lets put a path copy in the Heap (calling new), separating the path from the graph eliminated after!
	while (!bestState.getInitialState()){
		path.push_back(new State(bestState));
		bestState = graph[bestState.getParentGraphIndex()];
	}
	return path;
}

std::vector<State> SeqRRT::getGraph(){
	std::vector<State>  graphVector = std::vector<State>(graph, &graph[graphIterator - 1]);
	return graphVector;
}


char* SeqRRT::getSearchName(){
	return "SequentialRRT";
}