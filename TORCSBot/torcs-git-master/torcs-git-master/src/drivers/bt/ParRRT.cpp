#include "ParRRT.h"


ParRRT::ParRRT(State initialState, int nIterations, State* kernelGraph, tTrackSeg* kernelSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration, int numKernelBlocks, int numKernelThreadsPerBlock){
	this->nIterations = nIterations;
	this->initialState = new State(initialState);
	this->initialState->setMyGraphIndex(0);

	this->kernelGraph = kernelGraph;
	this->kernelSegArray = kernelSegArray;

	this->nTrackSegs = nTrackSegs;
	this->actionSimDeltaTime = actionSimDeltaTime;
	this->maxCarAcceleration = maxCarAcceleration;

	this->numKernelBlocks = numKernelBlocks;
	this->numKernelThreadsPerBlock = numKernelThreadsPerBlock;
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

	graph = Kernel::callKernel(kernelGraph, kernelSegArray, nTrackSegs, initialState, nIterations, numKernelBlocks, numKernelThreadsPerBlock, actionSimDeltaTime);

	//-------------------- CALC BEST NODE -----------------------------

	double maxCost = -1 * DBL_MAX; //force a change
	for (int i = 1; i < nIterations + 1; i++){
		State currentState = graph[i];
		if (currentState.getMyGraphIndex() == -1)  //xRand is still unassigned
			continue;
		double distFromStart = currentState.distFromStart;
		if (distFromStart > maxCost){
			maxCost = distFromStart;
			bestState = currentState;
		}
	}

	if (bestState.getMyGraphIndex() == -1){
		State initialStateCopy = State(*initialState);
		initialStateCopy.setInitialState(false);
		initialStateCopy.setParentGraphIndex(initialState->getMyGraphIndex());
		initialStateCopy.setLevelFromStart(2);
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


//std::vector<State*> ParRRT::search(){
// 
//	int bestPathSize;
//	pathArray = Kernel::callKernel(kernelGraph, kernelSegArray, nTrackSegs, initialState, nIterations, numKernelBlocks, numKernelThreadsPerBlock, actionSimDeltaTime);
//
//	std::vector<State*> path = std::vector<State*>(bestPathSize);
//	
//	path[0] = pathArray;
//	//to cope with the interface -.-
//	for (int i = 1; i < bestPathSize; i++){
//		path[i] = (State*)((unsigned int)(path[i-1])+sizeof(State));
//	}
//	return path;
//
//}


std::vector<State> ParRRT::getGraph(){
	std::vector<State>  graphVector; // = std::vector<State>(pathArray, &pathArray[2]);
	return graphVector;
}


char* ParRRT::getSearchName(){
	return "ParallelRRT";
}