#pragma once

#ifndef PARRRTSTAR_H
#define PARRRTSTAR_H

#include "Kernel.cuh"
#include "RRT.cuh"
#include "Heuristics.cuh"
#include <robottools.h>
#include <iostream>
#include <algorithm>    // std::transform
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
	int nTrackSegs;
	int nIterations;
	State* initialState;
	int forwardSegments;
	State* graph; //the returned search tree!


	//kernel vars
	State* kernelGraph;
	tPolarVel* kernelVelArray;
	tSimpleTrackSeg* kernelSegArray;
	int numKernelBlocks;
	int numKernelThreadsPerBlock;


public: //methods
	ParRRT(State initialState, int nIterations, State* kernelGraph, tPolarVel* kernelVelArray, tSimpleTrackSeg* kernelSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration, int numKernelBlocks, int numKernelThreadsPerBlock);
	~ParRRT();

	std::vector<State*> search();
	std::vector<State> getGraph();

	char* getSearchName();
};

#endif