#include "SeqRRTStar.h"

SeqRRTStar::SeqRRTStar(State* initialState,double nIterations){
	this->nIterations = nIterations;
	graph.push_back(initialState);
	std::srand(time(NULL));
}

State* SeqRRTStar::randomState(){
	return new State(
		         (2*std::rand() / RAND_MAX)-1,
		         ((PI * std::rand()) / RAND_MAX)-PI/2
				 );

}

double  SeqRRTStar::evaluateDistance(State s1, State s2){
	//MOCK!
	//Here is where the cost of a state (an action) is calculated
	return std::abs(std::rand()%10);
}

State* SeqRRTStar::nearestNeighbor(State state, std::vector<State*> graph){
	//MOCK!

	return graph[std::rand() % graph.size()];
}

std::vector<State> SeqRRTStar::nearestNeighbors(State state, std::vector<State*> graph){
	//MOCK!
	std::vector<State> neighbors = std::vector<State>();

	for (int i = 0; i < NEIGHBOR_SAMPLE_BOUNDARY; i++){
		neighbors.push_back(*graph[std::rand() % graph.size()]);
	}
	return neighbors;
}

bool SeqRRTStar::considerFinalState(State finalState){
	//NOT SURE IF NEEDED
	return true;
}

State* SeqRRTStar::generateRRT(){
	double maxDistance = 0;
	State* bestState;

	for (int k = 0; k < nIterations; k++){

		State* xRand = randomState();
		
		State* xNearest = nearestNeighbor(*xRand, graph);
		double xRandNearDistance = evaluateDistance(*xNearest, *xRand);

		double cMin = xNearest->getDistance() + xRandNearDistance;
		double xRandDistance = cMin;

		xRand->setParent(xNearest);
		xRand->setDistance(xRandDistance);
		graph.push_back(xRand);



		//std::cout << "nearest...: " << xNearest->toString() << std::endl;

		//std::cout << "xrand...: " << xRand->toString() << std::endl;
		
		
/*
		std::vector<State> nearNeighbors = nearestNeighbors(xRand, graph);

		State xMin = xNearest;*/


		////check if there is a better path
		//for (State xCurr : nearNeighbors){
		//	double cCurr = xCurr.getDistance() + evaluateDistance(xCurr, xRand);
		//	if (cCurr > cMin){
		//		xMin = xCurr;
		//		cMin = cCurr;
		//	}
		//}

		////reorder path to create a better path
		//for (State xCurr : nearNeighbors){
		//	//except xMin
		//	if (&xCurr==&xMin) continue;

		//	double cCurr = cMin + evaluateDistance(xCurr, xRand);
		//	if (cCurr > xCurr.getDistance()){
		//		xCurr.setParent(&xRand);
		//	}
		//}

		if (xRand->getDistance() > maxDistance){
			maxDistance = xRand->getDistance();
			bestState = xRand;
		}
		

	}


	std::cout << "escreve porco!: " << std::endl;

	for (std::vector<State*>::iterator i = graph.begin(); i != graph.end(); ++i)
	std::cout << (*i)->toString() << ' ' << std::endl;

	return bestState;
}


std::vector<State> SeqRRTStar::search(){
	std::vector<State> path = std::vector<State>();
	State* bestState = this->generateRRT();
	State* currState = bestState;

	std::cout << "best...: " << bestState->toString() << std::endl;


	while (currState->getParent() != NULL){
		path.push_back(*currState);
		currState = currState->getParent();
	}
	std::cout << "path...: " << std::endl;

	for (std::vector<State>::iterator i = path.begin(); i != path.end(); ++i)
		std::cout << i->toString() << ' ' << std::endl;


	return path;

}




