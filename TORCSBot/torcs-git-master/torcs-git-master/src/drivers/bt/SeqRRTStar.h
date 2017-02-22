#pragma once

#ifndef SEQRRTSTAR_H
#define SEQRRTSTAR_H

#include "RRTStar.cuh"
#include "Heuristics.cuh"
#include <robottools.h>
#include <iostream>

#include<time.h>

//-------------- Sequential Search class--------------------------
// In this version there are two types of cost:
// - the cost of the node represents the smoothness between the transition from the node to its son
// - the path cost is the distance travelled by the path along the track
//----------------------------------------------------------------

class SeqRRTStar : public RRTStar{
private: //vars

	//config vars
	int NEIGHBOR_SAMPLE_BOUNDARY = 4;
	int NEIGHBOR_DELTA_POS = 30;
	int NEIGHBOR_DELTA_SPEED = 10;
	double actionSimDeltaTime = 0.02 * 100; // assuming 100 game ticks action simulation
	double numberOfEmergencyCycles = 20;

	//initialization vars
	tTrackSeg currentSearchSeg;
	tTrackSeg forwardSearchSeg;
	tTrackSeg* trackSegArray;
	int nTrackSegs;
	tCarElt car;
	double nIterations;
	State* initialState;
	int forwardSegments;
	//Opponent *opponent;		// The array of opponents.

	//aux vars
	double maxPathCost;
	State* bestState;

	State** graph; // the RRT itself!
	unsigned int graphSize;
	int graphIterator;

private: //methods

	//graph operations
	void initGraph();
	void resizeGraph(unsigned int newSize);
	void deleteGraph();
	void pushBackToGraph(State* element);

	State* randomState(tTrackSeg* initialSeg, tTrackSeg* finalSeg);

	//selection operations
	State* nearestNeighbor(State* state, State** graph);
	State** nearestNeighbors(State* state, std::vector<State*> graph);


	//main loop procedure
	void generateStates(double nIterations);
	State* generateRRT();

public:
	SeqRRTStar(State initialState, double nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments);
	~SeqRRTStar();

	void updateCar(tCarElt car); //for multiple search calls
	std::vector<State*> search();

	std::vector<State*> getGraph(); //for debug purposes
};

#endif


//--------RECICLE BIN------------

//std::vector<State*> SeqRRTStar::nearestNeighbors(State* state, std::vector<State*> graph){
//	std::vector<State*> neighbors = std::vector<State*>();
//
//	State** auxGraph = graph;
//
//	int effectiveIterations = (auxGraph.size() -1 < NEIGHBOR_SAMPLE_BOUNDARY) ? (auxGraph.size() - 1) : NEIGHBOR_SAMPLE_BOUNDARY;
//
//	for (int i = 0; i < effectiveIterations; i++){
//		int index = std::rand() % auxGraph.size();
//		//remove initial node
//		if (state->getInitialState()){
//			auxGraph.erase(std::remove(auxGraph.begin(), auxGraph.end(), state), auxGraph.end());
//		}
//		State* currNearestNeighbor = nearestNeighbor(state, auxGraph);
//		neighbors.push_back(currNearestNeighbor);
//		auxGraph.erase(std::remove(auxGraph.begin(), auxGraph.end(), currNearestNeighbor), auxGraph.end());
//	}
//	return neighbors;
//}

//std::vector<State*> nearNeighbors = nearestNeighbors(xRand, graph);
//State* xMin = xNearest;
//
//
////check if there is a better path
//for (State* xCurr : nearNeighbors){
//	double cCurr = xCurr->getPathCost() + evaluatePathCost(xCurr, xRand);
//	if (cCurr > cMin){
//		xMin = xCurr;
//		cMin = cCurr;
//	}
//}
//
//xRand->setParent(xMin);
//xRand->setPathCost(cMin);
//
//
////reorder path to create a better path (it is not working now as it creates loops)
//for (State* xCurr : nearNeighbors){
//	//except xMin
//	if (xCurr == xMin || xRand->getParent() == NULL || xRand->getParent() != xCurr) 
//		continue;
//
//	double cCurr = xRand->getDistance() + evaluateDistance(xCurr, xRand);
//	if (cCurr > xCurr->getDistance()){
//		xCurr->setParent(xRand);
//		xCurr->setDistance(cCurr);
//	}
//
//	//missing costs update to the path!
//
//}