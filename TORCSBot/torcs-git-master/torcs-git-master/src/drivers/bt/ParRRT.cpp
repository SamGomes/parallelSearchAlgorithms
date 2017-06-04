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
	if (this->pathArray != nullptr){
		delete[] pathArray; //allocated on the kernel
	}
}


//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

std::vector<State*> ParRRT::search(){
 
	int bestPathSize;
	pathArray = Kernel::callKernel(bestPathSize, kernelGraph, kernelSegArray, nTrackSegs, initialState, nIterations, numKernelBlocks, numKernelThreadsPerBlock, actionSimDeltaTime);

	std::vector<State*> path = std::vector<State*>(bestPathSize);
	
	//to cope with the interface -.-
	for (int i = 0; i < bestPathSize; i++){
		path[i] = &pathArray[i];
	}
	return path;

}


std::vector<State> ParRRT::getGraph(){
	std::vector<State>  graphVector; // = std::vector<State>(pathArray, &pathArray[2]);
	return graphVector;
}


char* ParRRT::getSearchName(){
	return "ParallelRRT";
}