#pragma once

#ifndef SEQRRTSTAR_H
#define SEQRRTSTAR_H

#include "RRT.cuh"
#include "Heuristics.cuh"
#include <robottools.h>
#include <iostream>

#include<time.h>

//------------------ SeqRRT Search class--------------------------
// This class represents the sequential search procedure which  
// implements the RRT base class virtual methods
//----------------------------------------------------------------

class SeqRRT : public RRT{
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

	//aux vars
	double maxCost;
	State bestState;

	State* graph; // the returned search tree!
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

public: //methods
	SeqRRT(State initialState, int nIterations, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration);
	~SeqRRT();

	std::vector<State*> search();
	std::vector<State>  getGraph(); //for debug purposes

	char* getSearchName();

};

#endif