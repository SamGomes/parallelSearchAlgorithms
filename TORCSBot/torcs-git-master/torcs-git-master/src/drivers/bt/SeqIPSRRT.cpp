#include "SeqIPSRRT.h"


SeqIPSRRT::SeqIPSRRT(State initialState, int nIterations, int nMockedThreads, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration){
	this->maxCost = -1 * DBL_MAX; //force a change
	this->bestState = State();

	this->startSegIndex = startSegIndex;
	this->finalIndex = finalIndex;

	this->nIterations = nIterations;

	float iterationRatio = (float)nIterations / (float)nMockedThreads;
	int nPartialIterations = 0;
	nPartialIterations = ceilf(iterationRatio) == iterationRatio ? (int)iterationRatio : (int)iterationRatio + 1;
	if (nPartialIterations == 0) nPartialIterations++;

	this->nMockedThreads = nMockedThreads;
	this->nPartialIterations = nPartialIterations;
	this->initialState = initialState;

	initGraph();
	pushBackToGraph(this->initialState);

	std::srand(clock());
	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;

	this->actionSimDeltaTime = actionSimDeltaTime;
}
SeqIPSRRT::~SeqIPSRRT(){
	//as we do not need the graph anymore we just delete it! (commented because it is used on driver.cpp  for debug purposes)
	deleteGraph();
}

void SeqIPSRRT::initGraph(){
	graphSize = (unsigned int)nIterations + 1;
	graph = new State[graphSize]; //upper bound removes useless resizes
	graphIterator = 0;
}

void SeqIPSRRT::resizeGraph(unsigned int newSize){ //can be more usefull for partial searches (maybe future work)
	State* newGraph = new State[newSize];
	graphSize = newSize;

	for (int i = 0; i < graphIterator; i++){
		newGraph[i] = graph[i];
	}
	delete[] graph;
	graph = newGraph;
}
void SeqIPSRRT::deleteGraph(){
	delete[] graph;
}


void SeqIPSRRT::pushBackToGraph(State &element){
	if (graphIterator == graphSize){
		//graph limited exceeded, resize the graph
		resizeGraph(graphSize + (unsigned int)nIterations);
	}
	element.setMyGraphIndex(graphIterator); //sets the element index
	graph[graphIterator] = element;
	graphIterator++;
}


void SeqIPSRRT::generateStates(double nIterations){

	for (int k = 0; k < nPartialIterations; k++){
		State* statesToInsert = new State[graphSize];
		int statesToInsertIterator = 0;
		for (int j = 0; j < nMockedThreads; j++){
			State xRand;
			//--------------------------------------- generate random sample -----------------------------------------------
			xRand = RandomStateGenerators::uniformRandomState(trackSegArray, nTrackSegs, nullptr);
			//xRand = RandomStateGenerators::gaussianRandomState(trackSegArray, nTrackSegs, initialState.getVelocity());

			//---------------------------------------- select neighbor ----------------------------------------------------
			State xNearest = UtilityMethods::nearestNeighbor(xRand, graph, graphIterator);
			xRand.setParentGraphIndex(xNearest.getMyGraphIndex());
			xRand.setLevelFromStart(xNearest.getLevelFromStart() + 1);

			//----------------------------------------- constraint checking ------------------------------------------------
			//the delta application also checks if the trajectory is valid
			if (!DeltaFunctions::applyDelta(graph, &xRand, &xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
				continue;
			}

			//---------------------------------------- calculate best path --------------------------------------------------
			//the best state is the one that is furthest from the start lane
			tStateRelPos xRandLocalPos;
			UtilityMethods::SimpleRtTrackGlobal2Local(&xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0, xNearest.getLocalPos().segId);
			xRand.setLocalPos(xRandLocalPos);
			double distFromStart = UtilityMethods::getTrackCenterDistanceBetween(trackSegArray, nTrackSegs, &xRand, &initialState, 500) / xRand.getLevelFromStart();
			xRand.distFromStart = distFromStart;
			statesToInsert[statesToInsertIterator++] = xRand;

			
		}
		//merge graph with partial graph!
		for (int i = 0; i < statesToInsertIterator; ++i) {
			pushBackToGraph(statesToInsert[i]);
			if (statesToInsert[i].distFromStart > maxCost){
				maxCost = statesToInsert[i].distFromStart;
				bestState = statesToInsert[i];
			}
		}
	}
}

State SeqIPSRRT::generateRRT(){
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

std::vector<State*> SeqIPSRRT::search(){
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

std::vector<State> SeqIPSRRT::getGraph(){
	std::vector<State>  graphVector = std::vector<State>(graph, &graph[graphIterator - 1]);
	return graphVector;
}


char* SeqIPSRRT::getSearchName(){
	return "SequentialIPSRRT";
}