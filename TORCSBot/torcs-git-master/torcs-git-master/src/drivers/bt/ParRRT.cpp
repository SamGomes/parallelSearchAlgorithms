#include "ParRRT.h"


ParRRT::ParRRT(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration){
	this->nIterations = nIterations;
	this->initialState = new State(initialState);

	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->actionSimDeltaTime = actionSimDeltaTime;
	this->maxCarAcceleration = maxCarAcceleration;
}
ParRRT::~ParRRT(){
	if (this->initialState != nullptr){
		delete initialState;
	}
	if (this->graph != nullptr){
		delete[] graph; //allocated on the kernel
	}
}


//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

std::vector<State*> ParRRT::search(){
 
	State bestState;
	graph = Kernel::callKernel(trackSegArray, nTrackSegs, initialState, nIterations, actionSimDeltaTime);

	//-------------------- CALC BEST NODE -----------------------------
	double maxCost = -1 * DBL_MAX; //force a change
	for (int i = 1; i < nIterations+1; i++){
		State currentState = graph[i];
		if (currentState.getMyGraphIndex() == -1)  //xRand is still unassigned
			continue;
		double distFromStart = currentState.distFromStart;
		if (distFromStart > maxCost){
			maxCost = distFromStart;
			bestState = currentState;
		}
	}

	//if no path can be found just return a copy of the initial state! ;)
	if (bestState.getMyGraphIndex() == -1){
		State initialStateCopy = State(*initialState);
		initialStateCopy.setInitialState(false);
		initialStateCopy.setParentGraphIndex(initialState->getMyGraphIndex());
		bestState = initialStateCopy;
	}

	//---------------------- BACKTRACKING ----------------------------------
	std::vector<State*> path;
	//lets put a path copy in the Heap (calling new), separating the path from the graph eliminated after!
	while (!bestState.getInitialState()){
		path.push_back(new State(bestState));
		bestState = graph[bestState.getParentGraphIndex()];
	}
	return path;

}


std::vector<State> ParRRT::getGraph(){
	std::vector<State>  graphVector = std::vector<State>(graph, &graph[nIterations + 1]);
	return graphVector;
}


char* ParRRT::getSearchName(){
	return "ParallelRRT";
}