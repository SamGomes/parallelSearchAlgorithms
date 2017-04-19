
#include "Kernel.cuh"


CUDA_GLOBAL
void warmStart(int* f)
{
	*f = 0;
}

CUDA_GLOBAL
void CUDAProcedure(tTrackSeg* trackSegArray, int nTrackSegs, State* graph, int stateIterator,
	int numThreads, int graphSize, double maxCost, double actionSimDeltaTime){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;

	int offset = (stateIterator*numThreads + idx) + 1; //the initial state does not need this computation

	curandState_t curandState;


	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		idx, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);


	State initialState = graph[0];

	State xRand;

	//--------------------------------------- generate random sample -----------------------------------------------

	xRand = RandomStateGenerators::uniformRandomState(trackSegArray, nTrackSegs, &curandState);
	//xRand = RandomStateGenerators::gaussianRandomState(trackSegArray, nTrackSegs, startSegIndex, finalIndex, initialState.getVelocity());


	//---------------------------------------- select neighbor ----------------------------------------------------

	State xNearest = UtilityMethods::nearestNeighbor(xRand, graph, graphSize);
	xRand.setParentGraphIndex(xNearest.getMyGraphIndex());
	xRand.setLevelFromStart(xNearest.getLevelFromStart() + 1);

	//----------------------------------------- constraint checking ------------------------------------------------

	////if the acceleration is too much for the car to handle, prune the state
	///*if (abs(xRand.getVelocity().angle- xNearest.getVelocity().angle) > maxCarAcceleration.angle || abs(xRand.getVelocity().intensity-xNearest.getVelocity().intensity) > maxCarAcceleration.intensity){
	//	return;
	//}*/

	//the delta application also checks if the trajectory is valid
	if (!DeltaFunctions::applyDelta(&xRand, &xNearest, trackSegArray, nTrackSegs, actionSimDeltaTime)){
		return;
	}


	//the best state is the one that is furthest from the start lane
	tStateRelPos xRandLocalPos;
	UtilityMethods::SimpleRtTrackGlobal2Local(&xRandLocalPos, trackSegArray, nTrackSegs, xRand.getPos().x, xRand.getPos().y, 0);
	xRand.setLocalPos(xRandLocalPos);
	//double distFromStart = UtilityMethods::SimpleGetDistanceFromStart(xRandLocalPos); // / xRand.getLevelFromStart();
	double distFromStart = UtilityMethods::getTrackCenterDistanceBetween(trackSegArray, nTrackSegs, &xRand, &initialState, 500) / xRand.getLevelFromStart();
	xRand.distFromStart = distFromStart;

	xRand.setMyGraphIndex(offset);
	graph[offset]= xRand;


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
	
	// -- create a context -- kernel warmup ---------------------------------------------

	int *f = NULL;
	cudaMalloc(&f, sizeof(int));
	warmStart <<< 1, 1 >> >(f);
	cudaFree(f);

	cudaDeviceSynchronize();
	printf( "kernel error : %s" , cudaGetErrorString(cudaPeekAtLastError()) );


	//-----------------------------------------------------------------------------------

}

State* Kernel::callKernel(tTrackSeg* segArray, int nTrackSegs, State* initialState, int numIterations, double actionSimDeltaTime){

	int graphSize = numIterations + 1;

	State* graph = new State[(unsigned int)graphSize];
	initialState->setMyGraphIndex(0);
	graph[0] = *initialState;

	State* auxGraph;
	tTrackSeg* auxSegArray;

	double maxPathCost = 0; //just to mock (was not removed as it can still be needed)

	int NUM_BLOCKS = 1;
	int NUM_THREADS_EACH_BLOCK = 100;
	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;

	float iterationRatio = (float) numIterations / (float) NUM_THREADS;
	int numPartialIterations = 0;
	numPartialIterations = ceilf(iterationRatio) == iterationRatio ? (int) iterationRatio : (int) iterationRatio + 1;
	
	if (numPartialIterations == 0) numPartialIterations++;


	cudaMalloc(&auxGraph, sizeof(State)*(unsigned int)graphSize);
	cudaMalloc(&auxSegArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs);

	cudaMemcpy(auxSegArray, segArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs, cudaMemcpyHostToDevice);
	cudaMemcpy(auxGraph, graph, sizeof(State)*(unsigned int)graphSize, cudaMemcpyHostToDevice);

	for (int i = 0; i < numPartialIterations; i++)
	{
		CUDAProcedure << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> > (auxSegArray, nTrackSegs, auxGraph, i,
			NUM_THREADS, graphSize, maxPathCost, actionSimDeltaTime);
		cudaDeviceSynchronize();
		printf("kernel error: %s\n----------\n", cudaGetErrorString(cudaPeekAtLastError()));
	}
	
	cudaMemcpy(graph, auxGraph, sizeof(State)*(unsigned int)graphSize, cudaMemcpyDeviceToHost);

	cudaFree(auxGraph);
	cudaFree(auxSegArray);

	cudaDeviceSynchronize();
	return graph;
}
