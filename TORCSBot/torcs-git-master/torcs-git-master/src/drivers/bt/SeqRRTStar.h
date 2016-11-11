#ifndef NEIGHBOR_SAMPLE_BOUNDARY
#define NEIGHBOR_SAMPLE_BOUNDARY 20
#endif

#pragma once

#ifndef SEQRRTSTAR_H
#define SEQRRTSTAR_H

#include "State.cuh"
#include <queue>
#include <vector>
#include <iostream>

#include<time.h>

class SeqRRTStar{
	private:
		std::vector<State*> graph;
		State dynamicFinalState;
		double nIterations;
	public:
		SeqRRTStar(State* initialState, double nIterations);
		std::vector<State> search();
		State* generateRRT();
		double evaluateDistance(State s1, State s2);
		State* randomState();
		State* nearestNeighbor(State state, std::vector<State*> graph);
		std::vector<State> nearestNeighbors(State state, std::vector<State*> graph);
		bool considerFinalState(State finalState);
};

#endif

