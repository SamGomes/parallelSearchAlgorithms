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

	//initialization vars
	tPolarVel maxCarAcceleration;
	double actionSimDeltaTime; 
	int startSegIndex;
	int finalIndex;
	tTrackSeg* trackSegArray;
	int nTrackSegs;
	int nIterations;
	State initialState;
	//Opponent *opponent;		// The array of opponents.

	//aux vars
	double maxCost;
	State bestState;

	State* graph; // the RRT itself!
	unsigned int graphSize;
	int graphIterator;

private: //methods

	//graph operations
	void initGraph();
	void resizeGraph(unsigned int newSize);
	void deleteGraph();
	void pushBackToGraph(State &element);

	//main loop procedure
	void generateStates(double nIterations);
	State generateRRT();

public:
	SeqRRTStar(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration);
	~SeqRRTStar();

	std::vector<State*> search();
	std::vector<State>  getGraph(); //for debug purposes

	char* getSearchName();

};

#endif