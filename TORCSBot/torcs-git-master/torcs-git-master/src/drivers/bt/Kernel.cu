
#include "Kernel.cuh"


__global__ void warmStart(int* f)
{
	*f = 0;
}

__global__ void CUDAProcedure(tTrackSeg* segArray, int nTrackSegs, State* graph, int stateIterator, 
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex, 
	int numThreads, double maxPathCost, State* bestState,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;

	int offset = (stateIterator*numThreads + idx) + 1; //the initial state does not need this computation

	curandState_t curandState;


	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		idx, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);


	double trackMapXMin = minXVertex;
	double trackMapXMax = maxXVertex;

	double trackMapXDelta = trackMapXMax - trackMapXMin;

	double trackMapYMin = minYVertex;
	double trackMapYMax = maxYVertex;

	double trackMapYDelta = trackMapYMax - trackMapYMin;


	double trackMapZMin = 0;
	double trackMapZMax = 20;

	double trackMapZDelta = trackMapZMax - trackMapZMin;

	double minSpeed = 0;
	double maxSpeed = 60;

	double speedDelta = maxSpeed - minSpeed;


	double minAccel = 0;
	double maxAccel = 10;

	double accelDelta = maxAccel - minAccel;


	//------------------------generate random point --------------------------------

	double randPosX = trackMapXDelta * curand_uniform(&curandState) + trackMapXMin;
	double randPosY = trackMapYDelta * curand_uniform(&curandState) + trackMapYMin;
	double randPosZ = trackMapZDelta * curand_uniform(&curandState) + trackMapZMin;

	tPosd randPos;
	randPos.x = randPosX;
	randPos.y = randPosY;
	randPos.z = randPosZ;

	double randSpeedX = speedDelta * curand_uniform(&curandState) + minSpeed;
	double randSpeedY = speedDelta * curand_uniform(&curandState) + minSpeed;
	double randSpeedZ = speedDelta * curand_uniform(&curandState) + minSpeed;

	tPosd randSpeed;
	randSpeed.x = randSpeedX;
	randSpeed.y = randSpeedY;
	randSpeed.z = randSpeedZ;

	double randAccelX = accelDelta * curand_uniform(&curandState) + minAccel;
	double randAccelY = accelDelta * curand_uniform(&curandState) + minAccel;
	double randAccelZ = accelDelta * curand_uniform(&curandState) + minAccel;

	tPosd randAccel;
	randAccel.x = randAccelX;
	randAccel.y = randAccelY;
	randAccel.z = randAccelZ;

	State xRand = State(randPos, randSpeed, randAccel);

	//------------------------------find parent--------------------------------------

	//the generation didnt work
	if (!ConstraintChecking::validPoint(segArray, nTrackSegs, &xRand, (double) -2.0)){
		return;
	}
		
	State* xNearest = Kernel::nearestNeighbor(&xRand, graph, 2000, actionSimDeltaTime); //GRAPH ITERATOR FUCKUP!
	xRand.setParentGraphIndex(xNearest->getMyGraphIndex());
	printf("nearestIndex: %d\n", xNearest->getMyGraphIndex());

	//------------------------------apply delta--------------------------------------

	DeltaHeuristics::applyDelta(&xRand, xNearest, segArray, nTrackSegs, forwardSegments, neighborDeltaPos, neighborDeltaSpeed);


	double cMin = xNearest->getPathCost() + EvalFunctions::evaluatePathCost(segArray, nTrackSegs, xNearest, &xRand, forwardSegments); //redifine path cost for new coords
	xRand.setPathCost(cMin);

	//printf("parent out!:%f:%f\n", xRand.getPos().x, xRand.getParent()->getPos().x);

	////the delta application didnt work
	if (!ConstraintChecking::validPoint(segArray, nTrackSegs, &xRand, (double) -1.0)){
		return;
	}


	

	//------------------------------push to graph--------------------------------------
	xRand.setMyGraphIndex(offset);
	graph[offset] = xRand;
	


	
}

//This method takes off the CUDA initialization delay on first call during real-time
// because it is done during loading (pre-computing phase)
void Kernel::gpuWarmup(){
	int count;
	cudaDeviceProp prop;

	cudaGetDeviceCount(&count);

	// --- print devices info ----------------------------------------------------------

	for (int i = 0; i<count; i++) {

		cudaGetDeviceProperties(&prop, i);

		printf("--- General Information for device %d ---\n", i);
		printf("Name: %s\n", prop.name);
		printf("Compute capability: %d.%d\n", prop.major, prop.minor);
		printf("Clock rate: %d\n", prop.clockRate);
		printf("Device copy overlap: ");
		if (prop.deviceOverlap) printf("Enabled\n");
		else printf("Disabled\n");
		printf("Kernel execution timeout: ");
		if (prop.kernelExecTimeoutEnabled) printf("Enabled\n");
		else printf("Disabled\n");

		printf("--- Memory Information for device %d ---\n", i);
		printf("Total global mem: %ld\n", prop.totalGlobalMem);
		printf("Total constant mem: %ld\n", prop.totalConstMem);
		printf("Max mem pitch: %ld\n", prop.memPitch);
		printf("Texture alignment: %ld\n", prop.textureAlignment);

		printf("--- MP Information for device %d ---\n", i);
		printf("Multiprocessor count: %d\n", prop.multiProcessorCount);
		printf("Shared mem per mp: %ld\n", prop.sharedMemPerBlock);
		printf("Registers per mp: %d\n", prop.regsPerBlock);
		printf("Threads in warp: %d\n", prop.warpSize);
		printf("Max threads per block: %d\n", prop.maxThreadsPerBlock);
		printf("Max thread dimensions: (%d, %d, %d)\n", prop.maxThreadsDim[0],
			prop.maxThreadsDim[1],
			prop.maxThreadsDim[2]);
		printf("Max grid dimensions: (%d, %d, %d)\n", prop.maxGridSize[0],
			prop.maxGridSize[1],
			prop.maxGridSize[2]);

		printf("\n");
	}
	
	// -- create a context -- kernel warmup ---------------------------------------------

	int *f = NULL;
	cudaMalloc(&f, sizeof(int));
	warmStart <<< 1, 1 >> >(f);
	cudaFree(f);

	cudaDeviceSynchronize();
	std::cout << "kernel error: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;


	//-----------------------------------------------------------------------------------

}

State* Kernel::callKernel(tTrackSeg* segArray, int nTrackSegs, State* initialState,
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex,
	double numIterations,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime){

	State* auxBestState;

	State* graph = new State[(unsigned int)numIterations+1];
	initialState->setMyGraphIndex(0);
	graph[0] = *initialState;


	State* auxGraph;
	tTrackSeg* auxSegArray;

	double maxPathCost = 0; //just to mock (was not removed as it can still be needed)

	int NUM_BLOCKS = 5;
	int NUM_THREADS_EACH_BLOCK = 100;
	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;

	int numPartialIterations = numIterations / NUM_THREADS;

	if (numPartialIterations == 0) numPartialIterations++;


	clock_t mallocTimer;
	clock_t memcpyTimer1;
	clock_t kernelCallTimer;
	clock_t syncronizeTimer;
	clock_t memcpyTimer2;

	mallocTimer = clock();

	cudaMalloc(&auxGraph, sizeof(State)*(unsigned int)numIterations);
	cudaMalloc(&auxSegArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs);
	cudaMalloc(&auxBestState, sizeof(State));

	mallocTimer = clock() - mallocTimer;
	std::cout << "malloc timer: " << double(mallocTimer) / (double) CLOCKS_PER_SEC << std::endl;

	memcpyTimer1 = clock();

	//cudaMemcpy(auxBestPath, bestPath, sizeof(State)*(unsigned int)numIterations, cudaMemcpyHostToDevice);
	cudaMemcpy(auxSegArray, segArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs, cudaMemcpyHostToDevice);
	cudaMemcpy(auxGraph, graph, sizeof(State)*(unsigned int)numIterations, cudaMemcpyHostToDevice);

	memcpyTimer1 = clock() - memcpyTimer1;
	std::cout << "memcpy1 timer: " << double(memcpyTimer1) / (double) CLOCKS_PER_SEC << std::endl;

	for (int i = 0; i < numPartialIterations; i++)
	{
		kernelCallTimer = clock();

		CUDAProcedure << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> > (auxSegArray, nTrackSegs, auxGraph, i,
			minXVertex, maxXVertex, minYVertex, maxYVertex,
			NUM_THREADS, maxPathCost, auxBestState,
			forwardSegments, neighborDeltaPos, neighborDeltaSpeed, actionSimDeltaTime);

		kernelCallTimer = clock() - kernelCallTimer;

		syncronizeTimer = clock();

		cudaDeviceSynchronize();

		syncronizeTimer = clock() - syncronizeTimer;

		std::cout << "kernell call timer: " << double(kernelCallTimer) / (double) CLOCKS_PER_SEC << std::endl;
		std::cout << "sync timer: " << double(syncronizeTimer) / (double) CLOCKS_PER_SEC << std::endl;

	}
	
	memcpyTimer2 = clock();

	cudaMemcpy(graph, auxGraph, sizeof(State)*(unsigned int)numIterations, cudaMemcpyDeviceToHost);
	//cudaMemcpy(bestState, auxBestState, sizeof(State)*(unsigned int)numIterations, cudaMemcpyDeviceToHost);

	memcpyTimer2 = clock() - memcpyTimer2;

	std::cout << "memcpyTimer2 timer: " << double(memcpyTimer2) / (double) CLOCKS_PER_SEC << std::endl;


	cudaFree(auxGraph);
	cudaFree(auxSegArray);
	cudaFree(auxBestState);

	cudaDeviceSynchronize();
	std::cout << "kernel error: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;


	

	return graph;
}
