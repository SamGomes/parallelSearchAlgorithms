#include "ParRRTStar.h"


ParRRTStar::ParRRTStar(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration){

	this->nIterations = nIterations;
	this->initialState = new State(initialState);

	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->actionSimDeltaTime = actionSimDeltaTime;
	this->maxCarAcceleration = maxCarAcceleration;
}
ParRRTStar::~ParRRTStar(){
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

std::vector<State*> ParRRTStar::search(){

	 graph = Kernel::callKernel(trackSegArray, nTrackSegs, initialState, nIterations, actionSimDeltaTime);

	 State bestState;

	 //-------------------- CALC BEST NODE -----------------------------

	 double maxCost = -1 * DBL_MAX; //force a change

	 
	 for (int i = 1; i < nIterations+1; i++){
		 State currentState = graph[i];
		

		 if (currentState.getMyGraphIndex() == -1)  //xRand is still unassigned
			 continue;


		 ////the best state is the one that is furthest from the start lane
		 tTrkLocPos xRandLocalPos;
		 UtilityMethods::SimpleRtTrackGlobal2Local(&xRandLocalPos, trackSegArray, nTrackSegs, currentState.getPos().x, currentState.getPos().y, 0);
		 currentState.setLocalPos(xRandLocalPos);
		 //double distFromStart = UtilityMethods::SimpleGetDistanceFromStart(xRandLocalPos); // / xRand.getLevelFromStart();
		 double distFromStart = UtilityMethods::getTrackCenterDistanceBetween(trackSegArray, nTrackSegs, &currentState, initialState, 500) / currentState.getLevelFromStart();
		 //currentState.distFromStart = distFromStart;


		 if (distFromStart > maxCost){
			 maxCost = distFromStart;
			 bestState = currentState;
		 }
	 }

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


std::vector<State> ParRRTStar::getGraph(){
	std::vector<State>  graphVector = std::vector<State>(graph, &graph[nIterations - 1]);
	return graphVector;
}


char* ParRRTStar::getSearchName(){
	return "ParallelRRT";
}