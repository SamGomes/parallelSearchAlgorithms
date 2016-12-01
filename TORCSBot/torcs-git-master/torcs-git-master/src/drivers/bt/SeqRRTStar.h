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
		double NEIGHBOR_DELTA_POS = 100;
		double NEIGHBOR_DELTA_SPEED = 10;

		std::vector<State*> graph;
		State dynamicFinalState;
		double nIterations;

		// Track variable.
		tTrack track;
		//Opponent *opponent;		// The array of opponents.

		double actionSimDeltaTime = 0.02 * 100; //assigned for debug purposes (100 game ticks action simulation)


		bool validPoint(State* targetState);
		State* generateRRT();
		
		double evaluatePathCost(State* s1, State* s2);
		double computeSmoothness(tPosd initialPos, tPosd finalPos, tPosd initialSpeed, tPosd finalSpeed, tPosd acceleration, double simDeltaTime);
		double computeCost(tPosd initialPos, tPosd finalPos, tPosd initialSpeed, tPosd finalSpeed, tPosd acceleration, double simDeltaTime);
		
		State* randomState();
		State* nearestNeighbor(State* state, std::vector<State*> graph);
		std::vector<State*> nearestNeighbors(State* state, std::vector<State*> graph);
		bool considerFinalState(State* finalState);

		void normalizeState(State* state);
		double normalizeDiff(State* state, double num1, double num2, double delta);

		bool mostFar(tTrkLocPos l1, tTrkLocPos l2, int fwdLimit);

	public:
		SeqRRTStar(State* initialState, double nIterations, tTrack track);
		~SeqRRTStar();
		std::vector<State> search();
};

#endif

