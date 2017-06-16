
#include "Kernel.cuh"

__constant__ tPolarVel constVelArray[6401];


//------------------------------------------------------------------------------------------------------------
//--------                                    CUDA Kernels                                            --------
//------------------------------------------------------------------------------------------------------------

CUDA_GLOBAL
void graphInit(State* graph, tPolarVel* velArray, int numThreads, int graphSize){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	//initialize the graph
	int partialOffset = idx+1;
	int partialIterator = 0;
	velArray[0] = graph[0].getVelocity();
	while (partialOffset < (graphSize)){
		graph[partialOffset] = State();
		velArray[partialOffset].intensity = -1;
		partialIterator++;
		partialOffset = (partialIterator*numThreads + idx) + 1;
	}
	
}


CUDA_GLOBAL
void CUDAProcedure(tSimpleTrackSeg* trackSegArray, int nTrackSegs, State* graph, tPolarVel* kernelVelArray, int stateIterator,
	int numThreads, int graphSize, double actionSimDeltaTime){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	int offset = (stateIterator*numThreads + idx) + 1; //the initial state does not need this computation

	/*extern __shared__ tPolarVel sharedSegArray[];

	if (threadIdx.x == 0){
		for (int i = 0; i < graphSize; i++){
			sharedSegArray[i] = kernelVelArray[i];
		}
	}
	__syncthreads();*/

	curandState_t curandState;
	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		idx, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);

	State* initialState = &graph[0];
	State xRand;

	//--------------------------------------- generate random sample -----------------------------------------------
	xRand = RandomStateGenerators::uniformRandomState(nTrackSegs, &curandState);

	//---------------------------------------- select neighbor ----------------------------------------------------
	//State xNearest = UtilityMethods::nearestNeighbor(xRand, graph, graphSize);
	State xNearest = UtilityMethods::nearestNeighborKernel(xRand, constVelArray, graph, graphSize);
	xRand.setParentGraphIndex(xNearest.getMyGraphIndex());
	xRand.setLevelFromStart(xNearest.getLevelFromStart() + 1);

	//----------------------------------------- constraint checking ------------------------------------------------
	//the delta application also checks if the trajectory is valid
	if (!DeltaFunctions::applyDelta(graph, &xRand, &xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
		return;
	}
	//the best state is the one that is furthest from the start lane
	tStateRelPos xRandLocalPos;
	if (!UtilityMethods::SimpleRtTrackGlobal2Local(&xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0, xNearest.getLocalPos().segId))
		return;
	xRand.setLocalPos(xRandLocalPos);
	double distFromStart = UtilityMethods::getTrackCenterDistanceBetween(trackSegArray, nTrackSegs, &xRand, initialState, 500) / xRand.getLevelFromStart();
	xRand.distFromStart = distFromStart;

	xRand.setMyGraphIndex(offset);
	graph[offset]= xRand;
	kernelVelArray[offset] = xRand.getVelocity();
	//printf("bug:%d", offset);
}


//This kernel takes off the CUDA initialization delay on first call during real-time
// because it is done during loading (pre-computing phase). It also gathers and prints 
// some GPU information
CUDA_GLOBAL
void warmStart(int* f)
{
	*f = 0;
}


//------------------------------------------------------------------------------------------------------------
//--------                                  Invocation Methods                                        --------
//------------------------------------------------------------------------------------------------------------

void Kernel::gpuFree(State* kernelGraph, tPolarVel* kernelVelArray, tSimpleTrackSeg* kernelSegArray){
	cudaFree(kernelSegArray);
	cudaFree(kernelGraph);
	cudaFree(kernelVelArray);
}

void Kernel::gpuInit(State** kernelGraph, tPolarVel** kernelVelArray, tSimpleTrackSeg** kernelSegArray, int numIterations, tSimpleTrackSeg* segArray, int nTrackSegs){
	//----------------------- Print devices info -------------------------------
	int count;
	cudaDeviceProp prop;
	cudaGetDeviceCount(&count);

	cudaFuncSetCacheConfig(CUDAProcedure, cudaFuncCachePreferL1);

	int graphSize = numIterations + 1;

	for (int i = 0; i<count; i++) {
		cudaGetDeviceProperties(&prop, i);
		printf("---General  Information for device %d-----\n", i);
		printf("Name: %s\n", prop.name);
		printf("Compute capability: %d.%d\n", prop.major, prop.minor);
		printf("Clock rate: %d\n", prop.clockRate);
		printf("Device copy overlap: ");
		if (prop.deviceOverlap) printf("Enabled\n");
		else printf("Disabled\n");
		printf("Kernel execution timeout: ");
		if (prop.kernelExecTimeoutEnabled) printf("Enabled\n");
		else printf("Disabled\n");
		printf("---  Memory Information for device %d ---\n", i);
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

	//copy data to gpu
	cudaMalloc(kernelSegArray, sizeof(tSimpleTrackSeg)*(unsigned int)nTrackSegs);
	cudaMalloc(kernelGraph, graphSize*sizeof(State));
	cudaMemcpy(*kernelSegArray, segArray, sizeof(tSimpleTrackSeg)*(unsigned int)nTrackSegs, cudaMemcpyHostToDevice);
	cudaMalloc(kernelVelArray, graphSize*sizeof(tPolarVel));

	printf("init error: %s\n----------\n", cudaGetErrorString(cudaPeekAtLastError()));

	// ---------------- create a context -- kernel warmup -------------------------------
	int *f = NULL;
	cudaMalloc(&f, sizeof(int));
	warmStart <<< 1, 1 >> >(f);
	cudaFree(f);

	cudaDeviceSynchronize();
	printf( "warmup kernel error: %s" , cudaGetErrorString(cudaPeekAtLastError()) );
}



State* Kernel::callKernel(State* kernelGraph, tPolarVel* kernelVelArray, tSimpleTrackSeg* kernelSegArray, int nTrackSegs, State* initialState, int numIterations, int numBlocks, int numThreadsPerBlock, double actionSimDeltaTime){

	int graphSize = numIterations + 1;

	int NUM_BLOCKS = numBlocks;
	int NUM_THREADS_EACH_BLOCK = numThreadsPerBlock;
	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;

	float iterationRatio = (float)numIterations / (float)NUM_THREADS;
	int numPartialIterations = 0;
	numPartialIterations = ceilf(iterationRatio) == iterationRatio ? (int)iterationRatio : (int)iterationRatio + 1;
	if (numPartialIterations == 0) numPartialIterations++;

	cudaMemcpy(kernelGraph, initialState, sizeof(State), cudaMemcpyHostToDevice);

	graphInit << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> >(kernelGraph, kernelVelArray, NUM_THREADS, graphSize);


	for (int i = 0; i < numPartialIterations; i++)
	{
		cudaMemcpyToSymbol(constVelArray, kernelVelArray, sizeof(tPolarVel)*graphSize, 0, cudaMemcpyDeviceToDevice);
		//CUDAProcedure << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK, NUM_BLOCKS*graphSize*sizeof(tPolarVel) >> > (kernelSegArray, nTrackSegs, kernelGraph, kernelVelArray, i,
		//	NUM_THREADS, graphSize, actionSimDeltaTime); 
		CUDAProcedure << <NUM_BLOCKS, NUM_THREADS_EACH_BLOCK>> > (kernelSegArray, nTrackSegs, kernelGraph, kernelVelArray, i,
			NUM_THREADS, graphSize, actionSimDeltaTime);
		//printf("kernel error: %s\n------------\n", cudaGetErrorString(cudaPeekAtLastError()));
	}

	State* graph = new State[(unsigned int)graphSize];
	cudaMemcpy(graph, kernelGraph, sizeof(State)*(unsigned int)graphSize, cudaMemcpyDeviceToHost); //copy the graph back to main memory

	return graph;
}