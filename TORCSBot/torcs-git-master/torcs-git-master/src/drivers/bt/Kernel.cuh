#pragma once

#ifndef KERNEL_H
#define KERNEL_H

#include "State.cuh"

#include <time.h>
#include <stdio.h>
#include <iostream>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>
#include <vector>


#include <tgf.h>

__global__ void kernel();

std::vector<State> cuda_search(State initialState);

#endif
