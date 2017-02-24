
#include "Kernel.cuh"


__global__ void CUDAProcedure(tTrackSeg* segArray, int nTrackSegs, State* graph, int stateIterator, 
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex, 
	int numThreads, double maxPathCost, State* bestPath,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime){

	int idx = threadIdx.x + blockDim.x*blockIdx.x;

	curandState_t curandState;


	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		idx, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);

	int offset = stateIterator*numThreads + idx;

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
		
	State* xNearest = Kernel::nearestNeighbor(&xRand, graph, 200, actionSimDeltaTime); //GRAPH ITERATOR FUCKUP!
	xRand.setParent(xNearest);

	//------------------------------apply delta--------------------------------------

	DeltaHeuristics::applyDelta(&xRand, xNearest, segArray, nTrackSegs, forwardSegments, neighborDeltaPos, neighborDeltaSpeed);


	double cMin = xNearest->getPathCost() + EvalFunctions::evaluatePathCost(segArray, nTrackSegs, xNearest, &xRand, forwardSegments); //redifine path cost for new coords
	xRand.setPathCost(cMin);

	//printf("parent out!:%f:%f\n", xRand.getPos().x, xRand.getParent()->getPos().x);

	//the delta application didnt work
	if (!ConstraintChecking::validPoint(segArray, nTrackSegs, &xRand, (double) -2.0)){
		return;
	}


	//------------------------------push to graph--------------------------------------
	graph[offset] = xRand;

	//------------------------------check best path--------------------------------------

	if (xRand.getPathCost() >= maxPathCost){ //reevaluate best path

		maxPathCost = xRand.getPathCost();
		delete bestPath;
		int pathIndex = 0;
		State* initialNode = &graph[0];
		while (!xRand.getInitialState()){
			//printf("backtracking:%f\n", bestState->getPos().x);
			bestPath[pathIndex] = xRand;
			xRand = *xRand.getParent();
			pathIndex++;
		}
		bestPath[pathIndex] = *initialNode;
	}
	
}


State* Kernel::callKernel(tTrackSeg* segArray, int nTrackSegs,State* initialState,
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex,
	double numIterations,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime){

	State* bestPath = new State[(unsigned int)numIterations];
	State* auxBestPath;

	State* graph = new State[(unsigned int)numIterations];
	graph[0] = *initialState;


	State* auxGraph;
	tTrackSeg* auxSegArray;


	double maxPathCost = -1 * DBL_MAX; //force a change


	int NUM_BLOCKS = 3;
	int NUM_THREADS_EACH_BLOCK = 5;
	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;

	int numPartialIterations = numIterations / NUM_THREADS;

	if (numPartialIterations == 0) numPartialIterations++;


	cudaMalloc(&auxGraph, sizeof(State)*(unsigned int)numIterations);
	cudaMalloc(&auxSegArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs);
	cudaMalloc(&auxBestPath, sizeof(State)*(unsigned int)numIterations);

	cudaMemcpy(auxBestPath, bestPath, sizeof(State)*(unsigned int)numIterations, cudaMemcpyHostToDevice);
	cudaMemcpy(auxSegArray, segArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs, cudaMemcpyHostToDevice);
	cudaMemcpy(auxGraph, graph, sizeof(State)*(unsigned int)numIterations, cudaMemcpyHostToDevice);

	for (int i = 0; i < numPartialIterations; i++)
	{
		CUDAProcedure << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> > (auxSegArray, nTrackSegs, auxGraph, i,
			minXVertex, maxXVertex, minYVertex, maxYVertex,
			NUM_THREADS, maxPathCost, auxBestPath,
			forwardSegments, neighborDeltaPos, neighborDeltaSpeed, actionSimDeltaTime);

		cudaDeviceSynchronize();
	}
	

	cudaMemcpy(graph, auxGraph, sizeof(State)*(unsigned int)numIterations, cudaMemcpyDeviceToHost);
	cudaMemcpy(bestPath, auxBestPath, sizeof(State)*(unsigned int)numIterations, cudaMemcpyDeviceToHost);

	cudaDeviceSynchronize();

	/*for (int i = 0; i < 10; i++){
		printf("dgraphPos!:%f\n", graph[i].getPos().x);
	}*/

	cudaFree(auxGraph);
	cudaFree(auxSegArray);
	cudaFree(auxBestPath);

	std::cout << "gcnsjfddgjvhgdffgsgfuiserror: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;


	delete[] graph;

	return bestPath;
}
