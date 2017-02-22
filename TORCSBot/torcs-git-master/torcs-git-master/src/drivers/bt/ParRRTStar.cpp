#include "ParRRTStar.h"


ParRRTStar::ParRRTStar(State initialState, double nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments){

	this->maxPathCost = -1 * DBL_MAX; //force a change
	this->bestState = nullptr;
	this->currentSearchSeg = currentSearchSeg;

	this->forwardSegments = forwardSegments;

	int startSegIndex = (currentSearchSeg.id);
	int finalIndex = (startSegIndex + forwardSegments) % (nTrackSegs - 1);

	this->currentSearchSeg = trackSegArray[startSegIndex];
	this->forwardSearchSeg = trackSegArray[finalIndex];

	this->nIterations = nIterations;
	this->initialState = new State(initialState);

	initGraph();
	this->trackSegArray = trackSegArray;
	this->nTrackSegs = nTrackSegs;
	this->car = car;
}
ParRRTStar::~ParRRTStar(){

	//as we do not need the graph anymore we just delete it! (commented because it is used on driver.cpp  for debug purposes)
	deleteGraph();
}

void ParRRTStar::initGraph(){
	graphSize = (unsigned int)nIterations + 1;
	graph = new State*[graphSize]; //upper bound removes useless resizes
	graphIterator = 0;
}

void ParRRTStar::resizeGraph(unsigned int newSize){
	State** newGraph = new State*[newSize];
	graphSize = newSize;

	for (int i = 0; i < graphIterator; i++){
		newGraph[i] = graph[i];
	}
	delete[] graph;
	graph = newGraph;
}
void ParRRTStar::deleteGraph(){
	/*for (int i = 0; i < graphIterator; i++){
		delete graph[i];
	}*/

	delete[] graph;
}


void ParRRTStar::pushBackToGraph(State* element){
	if (graphIterator == graphSize){
		//graph limited exceeded, resize the graph
		resizeGraph(graphSize + (unsigned int)nIterations);
	}
	graph[graphIterator] = element;
	graphIterator++;
}

State* ParRRTStar::generateRandomStates(tTrackSeg* initialSeg, tTrackSeg* finalSeg){


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

	return callKernel(trackSegArray, nTrackSegs, minXVertex, maxXVertex, minYVertex, maxYVertex, nIterations,forwardSegments,NEIGHBOR_DELTA_POS,NEIGHBOR_DELTA_SPEED,actionSimDeltaTime);

}

//-------------------------------------------------------------
//----------------PUBLIC INTERFACE-----------------------------
//-------------------------------------------------------------

void ParRRTStar::updateCar(tCarElt car){
	this->car = car;
}
std::vector<State*> ParRRTStar::search(){

	State* CUDAGraph = generateRandomStates(&currentSearchSeg, &forwardSearchSeg);
	
	pushBackToGraph(this->initialState);


	/*for (int i = 0; i < nIterations; i++){
		std::cout << "foundNode: " << graph[i]->toString() << std::endl;
	}*/
	
	std::vector<State*> path;
	
	//lets put a path copy in the Heap (calling new), separating the path from the graph eliminated after!
	while (!bestState->getInitialState()){
		path.push_back(new State(*bestState));
		bestState = bestState->getParent();
	}

	
	return path;
}
std::vector<State*> ParRRTStar::getGraph(){
	std::vector<State*>  graphVector = std::vector<State*>(graph, &graph[graphIterator - 1]);

	return graphVector;
}