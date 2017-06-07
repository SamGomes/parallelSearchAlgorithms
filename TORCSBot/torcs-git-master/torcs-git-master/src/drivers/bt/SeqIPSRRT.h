#pragma once

#ifndef SEQIPSRRTSTAR_H
#define SEQIPSRRTSTAR_H

#include "RRT.cuh"
#include "Heuristics.cuh"
#include <robottools.h>
#include <iostream>

#include<time.h>


class SeqIPSRRT : public RRT{
private: //vars

	//initialization vars
	tPolarVel maxCarAcceleration;
	double actionSimDeltaTime;
	int startSegIndex;
	int finalIndex;
	tTrackSeg* trackSegArray;
	int nTrackSegs;
	int nIterations;
	int nPartialIterations;
	State initialState;

	//aux vars
	double maxCost;
	State bestState;

	int nMockedThreads;
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
	SeqIPSRRT(State initialState, int nIterations, int nMockedThreads, tTrackSeg* trackSegArray, int nTrackSegs, double actionSimDeltaTime, tPolarVel maxCarAcceleration);
	~SeqIPSRRT();

	std::vector<State*> search();
	std::vector<State>  getGraph(); //for debug purposes

	char* getSearchName();

};

#endif