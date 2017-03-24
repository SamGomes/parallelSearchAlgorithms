#include "ParRRTStar.h"


ParRRTStar::ParRRTStar(State initialState, int nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments, double actionSimDeltaTime){

	this->forwardSegments = forwardSegments;

	int startSegIndex = (currentSearchSeg.id);
	int finalIndex = (startSegIndex + forwardSegments) % (nTrackSegs - 1);

	this->currentSearchSegIndex = startSegIndex;
	this->forwardSearchSegIndex = finalIndex;

	this->nIterations = nIterations;
	this->initialState = new State(initialState);

	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->car = car;
}
ParRRTStar::~ParRRTStar(){
	if (this->initialState != nullptr){
		delete initialState;
	}
	if (this->graph != nullptr){
		delete[] graph; //allocated on the kernel
	}
}



State* ParRRTStar::generateSolution(int initialSegIndex, int finalSegIndex){


	double minXVertex = DBL_MAX;
	double maxXVertex = -1 * DBL_MAX;

	double minYVertex = DBL_MAX;
	double maxYVertex = -1 * DBL_MAX;

	/*double minXVertex = trackSegArray[initialSegIndex].vertex[2].x;
	double maxXVertex = trackSegArray[initialSegIndex].vertex[3].x;

	double minYVertex = trackSegArray[initialSegIndex].vertex[1].y;
	double maxYVertex = trackSegArray[initialSegIndex].vertex[0].y;*/

	//check if there are lower and higher bounds
	for (int i = initialSegIndex; i%nTrackSegs != finalSegIndex; i++){

		tTrackSeg currSeg = trackSegArray[i%nTrackSegs];


		if (currSeg.vertex[2].x < minXVertex){
			minXVertex = currSeg.vertex[2].x;
		}

		if (currSeg.vertex[3].x > maxXVertex){
			maxXVertex = currSeg.vertex[3].x;
		}

		if (currSeg.vertex[1].y < minYVertex){
			minYVertex = currSeg.vertex[1].y;
		}


		if (currSeg.vertex[0].y > maxYVertex){
			maxYVertex = currSeg.vertex[0].y;
		}


	}

	return Kernel::callKernel(trackSegArray, nTrackSegs, initialState, minXVertex, maxXVertex, minYVertex, maxYVertex, nIterations,forwardSegments,NEIGHBOR_DELTA_POS,NEIGHBOR_DELTA_SPEED,actionSimDeltaTime);
	//return callKernel(trackSegArray, nTrackSegs, minXVertex, maxXVertex, minYVertex, maxYVertex, nIterations);

}

//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

std::vector<State*> ParRRTStar::search(){

	 graph = generateSolution(currentSearchSegIndex, forwardSearchSegIndex);

	 State* bestState = nullptr;

	 //-------------------- CALC BEST NODE -----------------------------

	 double maxCost = -1 * DBL_MAX; //force a change

	 for (int i = 1; i < nIterations+1; i++){
		 State xRand = graph[i];
		 if (xRand.getMyGraphIndex() == -1)  //xRand is still unassigned
			 continue;
		 double distFromStart = UtilityMethods::SimpleGetDistanceFromStart(xRand.getLocalPos());
		 if (distFromStart > maxCost){
			 maxCost = distFromStart;
			 bestState = &xRand;
		 }
	 }

	 if (bestState == nullptr){
		 State* initialStateCopy = new State(*initialState);
		 initialStateCopy->setInitialState(false);
		 initialStateCopy->setParentGraphIndex(initialState->getMyGraphIndex());
		 bestState = initialStateCopy;
	 }
	//---------------------- BACKTRACKING ----------------------------------


	std::vector<State*> path;

	//lets put a path copy in the Heap (calling new), separating the path from the graph eliminated after!
	while (!bestState->getInitialState()){
		path.push_back(new State(*bestState));
		bestState = &graph[bestState->getParentGraphIndex()];
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