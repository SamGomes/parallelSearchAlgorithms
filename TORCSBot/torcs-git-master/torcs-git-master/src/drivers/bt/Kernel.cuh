#pragma once

#ifndef KERNEL_H
#define KERNEL_H


#include "State.cuh"
#include "Heuristics.cuh"
#include "StatsLogWriter.h"
#include <stdio.h>
#include <iostream>
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <device_functions.h>
#include <device_launch_parameters.h>

#pragma push_macro("tgfCUDAUncompatibleStuff")
#undef free
#undef malloc

CUDA_GLOBAL void graphInit(State* graph, tPolarVel* velArray, int numThreads, int graphSize);
CUDA_GLOBAL void CUDAProcedure(tSimpleTrackSeg* trackSegArray, int nTrackSegs, State* graph, tPolarVel* velArray, int stateIterator,
	int numThreads, int graphSize, double actionSimDeltaTime);
CUDA_GLOBAL void graphBacktrack(State* initialState, State* bestStates, int bestStatesSize, int* bestPathSize, State* graph);

class Kernel{
public:
	static void gpuFree(State* kernelGraph, tSimpleTrackSeg* kernelSegArray);
	static void gpuInit(State** kernelGraph, tSimpleTrackSeg** kernelSegArray, int numIterations, tSimpleTrackSeg* segArray, int nTrackSegs);
	static	State* callKernel(State* auxGraph, tSimpleTrackSeg* auxSegArray, int nTrackSegs, State* initialState, int numIterations, int numBlocks, int numThreadsPerBlock, double actionSimDeltaTime);
};

#pragma pop_macro("tgfCUDAUncompatibleStuff")

#endif
