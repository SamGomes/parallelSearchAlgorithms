
#include "Kernel.cuh"

//------------------------------------------------------------------------------------------------------------
//--------                                    CUDA Kernels                                            --------
//------------------------------------------------------------------------------------------------------------

CUDA_GLOBAL
void graphInit(State* graph, int numThreads, int graphSize, State* bestThreadStates){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	//initialize the graph
	int partialOffset = idx+1;
	int partialIterator = 0;
	while (partialOffset < (graphSize)){
		graph[partialOffset] = State();
		partialIterator++;
		partialOffset = (partialIterator*numThreads + idx) + 1;
	}
	//initialize partial best states array
	bestThreadStates[idx] = State();
	
}


CUDA_GLOBAL
void CUDAProcedure(tTrackSeg* trackSegArray, int nTrackSegs, State* graph, State* bestStates, int stateIterator,
	int numThreads, int graphSize, double actionSimDeltaTime){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	int offset = (stateIterator*numThreads + idx) + 1; //the initial state does not need this computation
	
	curandState_t curandState;
	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		idx, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);

	State* initialState = &graph[0];
	State xRand;

	//--------------------------------------- generate random sample -----------------------------------------------
	xRand = RandomStateGenerators::uniformRandomState(trackSegArray, nTrackSegs, &curandState);

	//---------------------------------------- select neighbor ----------------------------------------------------
	State xNearest = UtilityMethods::nearestNeighbor(xRand, graph, graphSize);
	xRand.setParentGraphIndex(xNearest.getMyGraphIndex());
	xRand.setLevelFromStart(xNearest.getLevelFromStart() + 1);

	//----------------------------------------- constraint checking ------------------------------------------------
	//the delta application also checks if the trajectory is valid
	if (!DeltaFunctions::applyDelta(&xRand, &xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
		return;
	}
	//the best state is the one that is furthest from the start lane
	tStateRelPos xRandLocalPos;
	UtilityMethods::SimpleRtTrackGlobal2Local(&xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0);
	xRand.setLocalPos(xRandLocalPos);
	double distFromStart = UtilityMethods::getTrackCenterDistanceBetween(trackSegArray, nTrackSegs, &xRand, initialState, 500) / xRand.getLevelFromStart();
	xRand.distFromStart = distFromStart;

	xRand.setMyGraphIndex(offset);
	graph[offset]= xRand;

	if (bestStates[idx].distFromStart < xRand.distFromStart){
		bestStates[idx] = xRand;
	}
}


CUDA_GLOBAL
void graphBacktrack(State* initialState, State* bestStates, int bestStatesSize, int* bestPathSize, State* graph){
	
	//-------------------- CALC BEST NODE -----------------------------
	State bestState;
	double maxCost = -1 * DBL_MAX; //force a change
	for (int i = 0; i < bestStatesSize; i++){
		State currentState = bestStates[i];
		if (currentState.getMyGraphIndex() == -1)  //xRand is still unassigned
			continue;
		double distFromStart = currentState.distFromStart;
		if (distFromStart > maxCost){
			maxCost = distFromStart;
			bestState = currentState;
		}
	}

	//if no path can be found just return a copy of the initial state! ;)
	if (bestState.getMyGraphIndex() == -1){
		State initialStateCopy = State(*initialState);
		initialStateCopy.setInitialState(false);
		initialStateCopy.setParentGraphIndex(initialState->getMyGraphIndex());
		initialStateCopy.setLevelFromStart(2);
		bestState = initialStateCopy;
		//printf("b2essfasastState. %d , %d\n---\n", initialStateCopy.getMyGraphIndex(), initialStateCopy.getLevelFromStart());
	}

	//printf("b2estState. %d , %d\n---\n", bestState.getMyGraphIndex(), bestState.getLevelFromStart());
	//free(bestStates);
	*bestPathSize = bestState.getLevelFromStart()-1;
	//bestStates = (State*)malloc(sizeof(State)*(*bestPathSize));
	//printf("size %d \n---\n", *bestPathSize);
	int bestPathIterator = 0;
	while (!bestState.getInitialState()){
		bestStates[bestPathIterator]=bestState;
		bestState = graph[bestState.getParentGraphIndex()];
		bestPathIterator++;
	}
/*

	printf("bestStates0: %d\n", bestStates[0].getMyGraphIndex());
	printf("bestStates1: %d\n", bestStates[1].getMyGraphIndex());
	printf("bestStates2: %d\n", bestStates[2].getMyGraphIndex());
	printf("bestStates3: %d\n", bestStates[3].getMyGraphIndex());*/
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

void Kernel::gpuFree(State* auxGraph, tTrackSeg* auxSegArray){
	cudaFree(auxSegArray); 
	cudaFree(auxGraph);
}

void Kernel::gpuInit(State** kernelGraph, tTrackSeg** kernelSegArray, int numIterations, tTrackSeg* segArray, int nTrackSegs){
	//----------------------- Print devices info -------------------------------
	int count;
	cudaDeviceProp prop;
	cudaGetDeviceCount(&count);

	int graphSize = numIterations + 1;

	for (int i = 0; i<count; i++) {
		cudaGetDeviceProperties(&prop, i);
		printf("---General  Information for device %d---\n", i);
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

	//copy segments to gpu
	cudaMalloc(kernelSegArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs);
	cudaMalloc(kernelGraph, sizeof(State)*(unsigned int)graphSize);
	cudaMemcpy(*kernelSegArray, segArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs, cudaMemcpyHostToDevice);

	printf("segs error: %s\n------------\n", cudaGetErrorString(cudaPeekAtLastError()));

	// ---------------- create a context -- kernel warmup -------------------------------
	int *f = NULL;
	cudaMalloc(&f, sizeof(int));
	warmStart <<< 1, 1 >> >(f);
	cudaFree(f);

	cudaDeviceSynchronize();
	printf( "warmup kernel error : %s" , cudaGetErrorString(cudaPeekAtLastError()) );
}

State* Kernel::callKernel(int& bestPathSize, State* kernelGraph, tTrackSeg* kernelSegArray, int nTrackSegs, State* initialState, int numIterations, int numBlocks, int numThreadsPerBlock, double actionSimDeltaTime){

	int graphSize = numIterations + 1;
	
	int NUM_BLOCKS = numBlocks;
	int NUM_THREADS_EACH_BLOCK = numThreadsPerBlock;
	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;

	State* bestThreadStates;
	int bestStatesSize = NUM_THREADS;

	int* kernelBestPathSize;
	
	float iterationRatio = (float) numIterations / (float) NUM_THREADS;
	int numPartialIterations = 0;
	numPartialIterations = ceilf(iterationRatio) == iterationRatio ? (int) iterationRatio : (int) iterationRatio + 1;
	if (numPartialIterations == 0) numPartialIterations++;

	cudaMalloc(&bestThreadStates, sizeof(State)*(unsigned int)bestStatesSize);
	cudaMemcpy(kernelGraph, initialState, sizeof(State), cudaMemcpyHostToDevice);

	graphInit << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> >(kernelGraph, NUM_THREADS, graphSize, bestThreadStates); cudaDeviceSynchronize();
	//printf("init error: %s\n------------\n", cudaGetErrorString(cudaPeekAtLastError()));
	for (int i = 0; i < numPartialIterations; i++)
	{
		CUDAProcedure << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> > (kernelSegArray, nTrackSegs, kernelGraph, bestThreadStates, i,
			NUM_THREADS, graphSize, actionSimDeltaTime);
		cudaDeviceSynchronize();
		//printf("kernel error: %s\n------------\n", cudaGetErrorString(cudaPeekAtLastError()));
	}

	cudaMalloc(&kernelBestPathSize, sizeof(int));
	graphBacktrack << < 1, 1 >> > (kernelGraph, bestThreadStates, bestStatesSize, kernelBestPathSize, kernelGraph); cudaDeviceSynchronize();
	cudaMemcpy(&bestPathSize, kernelBestPathSize, sizeof(int), cudaMemcpyDeviceToHost);
	//printf(" backtrack error: %s\n------------\n", cudaGetErrorString(cudaPeekAtLastError()));
	State* bestPath = new State[(unsigned int)bestPathSize];
	cudaMemcpy(bestPath, bestThreadStates, sizeof(State)*(unsigned int)bestPathSize, cudaMemcpyDeviceToHost); //copy the graph back to main memory

	cudaFree(bestThreadStates);
	cudaFree(kernelBestPathSize);

	cudaDeviceSynchronize();
	return bestPath;
}


//State* Kernel::callKernel(State* auxGraph, tTrackSeg* auxSegArray, int nTrackSegs, State* initialState, int numIterations, int numBlocks, int numThreadsPerBlock, double actionSimDeltaTime){
//
//	int graphSize = numIterations + 1;
//
//	double maxPathCost = 0; //just to mock (can be used in future work to optimize the search)
//
//	int NUM_BLOCKS = numBlocks;
//	int NUM_THREADS_EACH_BLOCK = numThreadsPerBlock;
//	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;
//
//	float iterationRatio = (float)numIterations / (float)NUM_THREADS;
//	int numPartialIterations = 0;
//	numPartialIterations = ceilf(iterationRatio) == iterationRatio ? (int)iterationRatio : (int)iterationRatio + 1;
//	if (numPartialIterations == 0) numPartialIterations++;
//
//	cudaMemcpy(auxGraph, initialState, sizeof(State), cudaMemcpyHostToDevice);
//
//	graphInit << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> >(auxGraph, NUM_THREADS, graphSize);
//
//	for (int i = 0; i < numPartialIterations; i++)
//	{
//		CUDAProcedure << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> > (auxSegArray, nTrackSegs, auxGraph, i,
//			NUM_THREADS, graphSize, maxPathCost, actionSimDeltaTime);
//		cudaDeviceSynchronize();
//		printf("kernel error: %s\n------------\n", cudaGetErrorString(cudaPeekAtLastError()));
//	}
//
//	State* graph = new State[(unsigned int)graphSize];
//	cudaMemcpy(graph, auxGraph, sizeof(State)*(unsigned int)graphSize, cudaMemcpyDeviceToHost); //copy the graph back to main memory
//
//	cudaDeviceSynchronize();
//	return graph;
//}