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
	tTrackSeg currentSearchSeg;
	tTrackSeg forwardSearchSeg;
	tTrackSeg* trackSegArray;
	int nTrackSegs;
	tCarElt car;
	double nIterations;
	State* initialState;
	int forwardSegments;

	State* graph;

private: //methods


	State* generateSolution(tTrackSeg* initialSeg, tTrackSeg* finalSeg);


public:
	ParRRTStar(State initialState, double nIterations, tCarElt car, tTrackSeg* trackSegArray, int nTrackSegs, tTrackSeg currentSearchSeg, int forwardSegments);
	~ParRRTStar();

	void updateCar(tCarElt car); //for multiple search calls
	std::vector<State*> search();

	std::vector<State*> getGraph(); //for debug purposes
};

#endif