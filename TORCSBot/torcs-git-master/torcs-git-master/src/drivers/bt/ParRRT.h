#pragma once

#ifndef PARRRTSTAR_H
#define PARRRTSTAR_H

#include "Kernel.cuh"
#include "RRT.cuh"
#include "Heuristics.cuh"
#include <robottools.h>
#include <iostream>

#include<time.h>

//----------------- ParRRT Search class---------------------------
// This class represents the GPGPU parallel search procedure which  
// implements the RRT base class virtual methods. This class the
// calls the CUDA kernel in the search() method
//----------------------------------------------------------------

class ParRRT : public RRT{
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

	State* graph; //the returned search tree!

public: //methods
	ParRRT(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration);
	~ParRRT();

	std::vector<State*> search();
	std::vector<State> getGraph();

	char* getSearchName();
};

#endif