#pragma once

#ifndef KERNEL_H
#define KERNEL_H



#include "State.cuh"
#include "Heuristics.cuh"
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <device_functions.h>
#include <device_launch_parameters.h>
#include <vector>


#include <tgf.h>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}




__global__ void CUDAProcedure(tTrackSeg* segArray, int nTrackSegs, State* graph, int stateIterator,
								double minXVertex, double maxXVertex, double minYVertex, double maxYVertex,
								int numThreads, double maxPathCost, State* bestPath,
								int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime);

class Kernel{
public:

	CUDA_HOSTDEV //the nearest point is the one in which its finalPos prediction ajusts to the current pos
	static State* nearestNeighbor(State* state, State* graph, int graphIterator, double actionSimDeltaTime){

		State* closestState = &graph[0];
		double minCost = DBL_MAX;
		for (int j = 0; j < graphIterator; j++){
			State* i = &graph[j];
			if (i == nullptr)
				continue;
			double currCost = EvalFunctions::evaluateStateCost(i, state, actionSimDeltaTime);
			if (minCost > currCost){
				minCost = currCost;
				closestState = i;
			}
		}
		return closestState;

	}

	static	State* callKernel(tTrackSeg* segArray, int nTrackSegs, State* initialState,
							double minXVertex, double maxXVertex, double minYVertex, double maxYVertex,
							double numIterations,
							int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime);

};



#endif
