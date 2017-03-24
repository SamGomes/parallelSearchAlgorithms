#pragma once

#ifndef PARRRTSTAR_H
#define PARRRTSTAR_H

#include "Kernel.cuh"
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

class ParRRTStar : public RRTStar{
private: //vars

	//config vars
	int NEIGHBOR_SAMPLE_BOUNDARY = 4;
	int NEIGHBOR_DELTA_POS = 30;
	int NEIGHBOR_DELTA_SPEED = 10;
	double actionSimDeltaTime = 0.02 * 100; // assuming 100 game ticks action simulation
	double numberOfEmergencyCycles = 20;

	//initialization vars
	int currentSearchSegIndex;
	int forwardSearchSegIndex;
	tTrackSeg* trackSegArray;
	int nTrackSegs;
	tCarElt car;
	int nIterations;
	State* initialState;
	int forwardSegments;

	State* graph;

private: //methods

	State* generateSolution(int initialSegIndex, int finalSegIndex);


public:
	ParRRTStar(State initialState, int nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments, double actionSimDeltaTime);
	~ParRRTStar();

	std::vector<State*> search();

	std::vector<State> getGraph(); //for debug purposes

	char* getSearchName();
};

#endif