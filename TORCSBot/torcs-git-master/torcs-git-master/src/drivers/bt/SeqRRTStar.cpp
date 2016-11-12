#include "SeqRRTStar.h"

SeqRRTStar::SeqRRTStar(State* initialState,double nIterations){
	this->nIterations = nIterations;
	graph.push_back(initialState);
	std::srand(time(NULL));
}

State* SeqRRTStar::randomState(){

	double rand = 2 * ((double)std::rand() / (double)RAND_MAX) - 1;
	return new State(
		         rand,
				 ((PI * std::rand()) / (double) RAND_MAX) - PI / 2
				 );

}

double  SeqRRTStar::evaluateDistance(State* s1, State* s2){
	//MOCK!
	//Here is where the cost of a state (an action) is calculated
	return std::abs(std::rand()%10);
}

State* SeqRRTStar::nearestNeighbor(State* state, std::vector<State*> graph){
	//MOCK!

	return graph[std::rand() % graph.size()];
}

std::vector<State*> SeqRRTStar::nearestNeighbors(State* state, std::vector<State*> graph){
	//MOCK! tem de ser alterado
	std::vector<State*> neighbors = std::vector<State*>();

	std::vector<State*> auxGraph = graph;

	int effectiveIterations = (auxGraph.size() -2 < NEIGHBOR_SAMPLE_BOUNDARY) ? (auxGraph.size() - 2) : NEIGHBOR_SAMPLE_BOUNDARY;

	for (int i = 0; i < effectiveIterations; i++){

		int index = std::rand() % auxGraph.size();
		//remove initial node and testing node
		while (auxGraph[index] == state || state->getParent()==NULL){
			index = std::rand() % auxGraph.size();
		}
		neighbors.push_back(auxGraph[index]);
		auxGraph.erase(auxGraph.begin() + index);
	}
	return neighbors;
}

bool SeqRRTStar::considerFinalState(State* finalState){
	//NOT SURE IF NEEDED
	return true;
}

State* SeqRRTStar::generateRRT(){
	double maxDistance = 0;
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

		xRand->setParent(xMin);
		xRand->setDistance(cMin);
		
		//reorder path to create a better path (it is not working now as it creates loops)
		//for (State* xCurr : nearNeighbors){
		//	//except xMin
		//	if (xCurr == xMin) continue;

		//	double cCurr = xRand->getDistance() + evaluateDistance(xCurr, xRand);
		//	if (cCurr > xCurr->getDistance() && xRand->getParent() != xCurr){
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


	//std::cout << "escrever RRT: " << std::endl;

	//for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i)
	//std::cout << (*i)->toString() << ' ' << std::endl;

	return bestState;
}


std::vector<State> SeqRRTStar::search(){
	std::vector<State> path = std::vector<State>();
	State* bestState = this->generateRRT();
	State* currState = bestState;

	while (currState->getParent() != NULL){
		std::cout << currState->toString() << std::endl;
		path.push_back(*currState);
		currState = currState->getParent();
	}
	

	return path;

}




