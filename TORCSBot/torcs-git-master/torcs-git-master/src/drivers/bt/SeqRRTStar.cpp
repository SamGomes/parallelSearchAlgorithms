#include "SeqRRTStar.h"


static bool compareStates(State* s1, State* s2) {
	double s1D = s1->getDistance();
	double s2D = s2->getDistance();
	return (s1D > s2D);
}


SeqRRTStar::SeqRRTStar(State* initialState, double nIterations){
	this->nIterations = nIterations;
	graph.push_back(initialState);
	std::srand(time(NULL));
}


SeqRRTStar::~SeqRRTStar(){
	//for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i)
		//delete (*i);
	//delete[] &graph;
}


State* SeqRRTStar::randomState(){

	double randPos = 1.1 * ((double)std::rand() / (double)RAND_MAX) - 0.1;
	double randSteer = 0.2 * (((PI * std::rand()) / (double)RAND_MAX) - PI / 2);
	return new State(randPos,randSteer); 

}

double MOCK = 0;

double  SeqRRTStar::evaluateDistance(State* s1, State* s2){
	//Here is where the cost of a state (an action) is calculated

	//return (s2->getFinalPos() - s1->getInitialPos()) / (0.5*s1->getAcceleration()*s1->getAcceleration() + s1->get)


	//MOCK!
	return std::abs(std::rand() % 10);
}

State* SeqRRTStar::nearestNeighbor(State* state, std::vector<State*> graph){
	std::sort(graph.begin(), graph.end(), compareStates);

	return graph.back();

	//MOCK IT!
	//return graph[std::rand() % graph.size()];
}

std::vector<State*> SeqRRTStar::nearestNeighbors(State* state, std::vector<State*> graph){
	std::vector<State*> neighbors = std::vector<State*>();

	std::vector<State*> auxGraph = graph;

	int effectiveIterations = (auxGraph.size() -2 < NEIGHBOR_SAMPLE_BOUNDARY) ? (auxGraph.size() - 2) : NEIGHBOR_SAMPLE_BOUNDARY;

	for (int i = 0; i < effectiveIterations; i++){

		int index = std::rand() % auxGraph.size();
		//remove initial node and testing node
		if (auxGraph[index] == state || state->getParent()==NULL){
			auxGraph.erase(std::remove(auxGraph.begin(), auxGraph.end(), state), auxGraph.end());
		}
		State* currNearestNeighbor = nearestNeighbor(state, auxGraph);
		neighbors.push_back(currNearestNeighbor);
		auxGraph.erase(std::remove(auxGraph.begin(), auxGraph.end(), currNearestNeighbor), auxGraph.end());
	}
	return neighbors;
}

bool SeqRRTStar::considerFinalState(State* finalState){
	//NOT SURE IF NEEDED
	return true;
}

State* SeqRRTStar::generateRRT(){
	double maxDistance = -1; //force a change
	State* bestState;

	for (int k = 0; k < nIterations; k++){

		State* xRand = randomState();
		
		State* xNearest = nearestNeighbor(xRand, graph);
		double xRandNearDistance = evaluateDistance(xNearest, xRand);

		double cMin = xNearest->getDistance() + xRandNearDistance;

		xRand->setParent(xNearest);
		xRand->setDistance(cMin);
		graph.push_back(xRand);


		std::vector<State*> nearNeighbors = nearestNeighbors(xRand, graph);

		State* xMin = xNearest;


		//check if there is a better path
		for (State* xCurr : nearNeighbors){
			double cCurr = xCurr->getDistance() + evaluateDistance(xCurr, xRand);
			if (cCurr > cMin){
				xMin = xCurr;
				cMin = cCurr;
			}
		}

		//xRand->setParent(xMin);
		//xRand->setDistance(cMin);
		//
		////reorder path to create a better path (it is not working now as it creates loops)
		//for (State* xCurr : nearNeighbors){
		//	//except xMin
		//	if (xCurr == xMin || xRand->getParent() == NULL || xRand->getParent() != xCurr) 
		//		continue;

		//	double cCurr = xRand->getDistance() + evaluateDistance(xCurr, xRand);
		//	if (cCurr > xCurr->getDistance()){
		//		xCurr->setParent(xRand);
		//		xCurr->setDistance(cCurr);
		//	}

		//	//missing costs update to the path!

		//}


		if (xRand->getDistance() > maxDistance){
			maxDistance = xRand->getDistance();
			bestState = xRand;
		}


	}
	return bestState;
}


std::vector<State> SeqRRTStar::search(){
	std::vector<State> path = std::vector<State>();
	State* bestState = this->generateRRT();
	State* currState = bestState;

	while (currState->getParent() != NULL){
		path.push_back(*currState);
		currState = currState->getParent();
	}

	return path;

}




