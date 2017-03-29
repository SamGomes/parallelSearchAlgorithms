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

	//initialization vars
	tPolarVel maxCarAcceleration;
	double actionSimDeltaTime;
	int currentSearchSegIndex;
	int forwardSearchSegIndex;
	tTrackSeg* trackSegArray;
	int nTrackSegs;
	int nIterations;
	State* initialState;
	int forwardSegments;

	State* graph;

public:
	ParRRTStar(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration);
	~ParRRTStar();

	std::vector<State*> search();

	std::vector<State> getGraph(); //for debug purposes

	char* getSearchName();
};

#endif