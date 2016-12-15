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
		double NEIGHBOR_DELTA_POS = 30;
		double NEIGHBOR_DELTA_SPEED = 10;

		double nCalls = 0;

		State* initialState;

		bool deleteBestState;
		State* bestState;

		std::vector<State*> graph;

		State dynamicFinalState;
		double nIterations;

		int forwardSegments;

		tTrack track; // Track variable.
		tCarElt car; //car variable
		
		tTrackSeg currentSearchSeg; //current car segment
		tTrackSeg forwardSearchSeg;

		//Opponent *opponent;		// The array of opponents.

		double actionSimDeltaTime = 0.02 * 100; //assigned for debug purposes (100 game ticks action simulation)


		bool validPoint(State* targetState, double distFromSides);
		State* generateRRT();
		
		double evaluateCost(State* state);
		double evaluatePathCost(State* s1, State* s2);
		
		State* randomState(tTrackSeg* initialSeg, tTrackSeg* finalSeg);
		State* nearestNeighbor(State* state, std::vector<State*> graph);
		std::vector<State*> nearestNeighbors(State* state, std::vector<State*> graph);
		bool considerFinalState(State* finalState);

		void normalizeState(State* state, State* parent);

		double localDistance(State* s1, State* s2, int fwdLimit);

	public:
		SeqRRTStar(State initialState, double nIterations, tCarElt car, tTrack track, tTrackSeg currentSearchSeg, int forwardSegments);
		~SeqRRTStar();
		std::vector<State*> search();

		std::vector<State*> getGraph(); //for debug purposes

};

#endif

