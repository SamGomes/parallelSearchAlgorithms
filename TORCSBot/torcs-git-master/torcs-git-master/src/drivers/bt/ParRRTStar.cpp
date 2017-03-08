#include "ParRRTStar.h"


ParRRTStar::ParRRTStar(State initialState, double nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments){

	
	this->currentSearchSeg = currentSearchSeg;

	this->forwardSegments = forwardSegments;

	int startSegIndex = (currentSearchSeg.id);
	int finalIndex = (startSegIndex + forwardSegments) % (nTrackSegs - 1);

	this->currentSearchSeg = trackSegArray[startSegIndex];
	this->forwardSearchSeg = trackSegArray[finalIndex];

	this->nIterations = nIterations;
	this->initialState = new State(initialState);

	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->car = car;
}
ParRRTStar::~ParRRTStar(){
	delete[] graph; //allocated on the kernel
}



State* ParRRTStar::generateSolution(tTrackSeg* initialSeg, tTrackSeg* finalSeg){


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

	return Kernel::callKernel(trackSegArray, nTrackSegs, initialState, minXVertex, maxXVertex, minYVertex, maxYVertex, nIterations,forwardSegments,NEIGHBOR_DELTA_POS,NEIGHBOR_DELTA_SPEED,actionSimDeltaTime);
	//return callKernel(trackSegArray, nTrackSegs, minXVertex, maxXVertex, minYVertex, maxYVertex, nIterations);

}

//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

void ParRRTStar::updateCar(tCarElt car){
	this->car = car;
}
std::vector<State*> ParRRTStar::search(){

	 graph = generateSolution(&currentSearchSeg, &forwardSearchSeg);

	 State* bestState = nullptr;

	 //-------------------- calc best node -----------------------------

	 double maxPathCost = -1 * DBL_MAX; //force a change

	 for (int i = 1; i < nIterations+1; i++){
		 State xRand = graph[i];
		 if (xRand.getMyGraphIndex() == -1)  //still unassigned
			 continue;
		 if (xRand.getPathCost() > maxPathCost){
			 maxPathCost = xRand.getPathCost();
			 bestState = &graph[i];
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
std::vector<State*> ParRRTStar::getGraph(){
	std::vector<State*>  graphVector; //= std::vector<State>(graph, &graph[graphIterator - 1]);

	return graphVector;
}