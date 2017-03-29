#pragma once

#ifndef KERNEL_H
#define KERNEL_H



#include "State.cuh"
#include "Heuristics.cuh"
#include "StatsLogWriter.h"
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

CUDA_GLOBAL void CUDAProcedure(tTrackSeg* trackSegArray, int nTrackSegs, State* graph, int stateIterator,
							  int numThreads, int graphSize, double maxCost, State* bestState, double actionSimDeltaTime);

class Kernel{
public:


	static void gpuWarmup();

	static	State* callKernel(tTrackSeg* segArray, int nTrackSegs, State* initialState, int numIterations, double actionSimDeltaTime);

};



#endif
