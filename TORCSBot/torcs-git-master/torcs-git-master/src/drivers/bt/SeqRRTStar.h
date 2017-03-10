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
	int nIterations;
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
	//State** nearestNeighbors(State* state, std::vector<State*> graph);


	//main loop procedure
	void generateStates(double nIterations);
	State* generateRRT();

public:
	SeqRRTStar(State initialState, int nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments);
	~SeqRRTStar();

	void updateCar(tCarElt car); //for multiple search calls
	std::vector<State*> search();

	std::vector<State*>  getGraph(); //for debug purposes

	char* getSearchName();

};

#endif