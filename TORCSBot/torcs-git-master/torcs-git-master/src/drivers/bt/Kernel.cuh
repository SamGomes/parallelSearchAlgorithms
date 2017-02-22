#pragma once

#ifndef KERNEL_H
#define KERNEL_H



#include "State.cuh"
#include "device_launch_parameters.h"
#include "Heuristics.cuh"
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
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


__global__ void Kernel(tTrackSeg* segArray, int nTrackSegs, State* graph,
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex,
	int numPartialIterations, double randState,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime);

State* callKernel(tTrackSeg* segArray, int nTrackSegs, 
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex, 
	double numIterations,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime);

#endif
