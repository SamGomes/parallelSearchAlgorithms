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

//#pragma push_macro("tgfCUDAUncompatibleStuff")
//#undef free
//#undef malloc

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{
	if (code != cudaSuccess)
	{
		fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
		if (abort) exit(code);
	}
}

CUDA_GLOBAL void CUDAProcedure(tTrackSeg* trackSegArray, int nTrackSegs, State* initialState, State* graph, int stateIterator,
							  int numThreads, int graphSize, double maxCost, State* bestState, double actionSimDeltaTime);

class Kernel{
public:
	static void gpuFree(State* auxGraph, tTrackSeg* auxSegArray);
	static void gpuInit(State** auxGraph, tTrackSeg** auxSegArray, int numIterations, tTrackSeg* segArray, int nTrackSegs);
	static	State* callKernel(State* auxGraph, tTrackSeg* auxSegArray, int nTrackSegs, State* initialState, int numIterations, int numBlocks, int numThreadsPerBlock, double actionSimDeltaTime);
};

//#pragma pop_macro("tgfCUDAUncompatibleStuff")

#endif
