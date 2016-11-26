#pragma once

#ifndef SEQRRTSTAR_H
#define SEQRRTSTAR_H

#include "State.cuh"
#include <robottools.h>
#include <queue>
#include <vector>
#include <iostream>

#include<time.h>

class SeqRRTStar{
	private:

		int NEIGHBOR_SAMPLE_BOUNDARY = 4;
		double NEIGHBOR_DELTA_PEDALPOS = 0.2;
		double NEIGHBOR_DELTA_STEERANGLE = 0.1;

		std::vector<State*> graph;
		State dynamicFinalState;
		double nIterations;

		// Track variable.
		tTrack track;
		//Opponent *opponent;		// The array of opponents.


		bool validPoint(tPosd target);
		State* generateRRT();
		double evaluatePathCost(State* s1, State* s2);
		State* randomState();
		State* nearestNeighbor(State* state, std::vector<State*> graph);
		std::vector<State*> nearestNeighbors(State* state, std::vector<State*> graph);
		bool considerFinalState(State* finalState);

		void normalizeState(State* state);
		double normalizeDiff(double num1, double num2, double delta);

	public:
		SeqRRTStar(State* initialState, double nIterations, tTrack track);
		~SeqRRTStar();
		std::vector<State> search();
};

#endif

