
#include "Kernel.cuh"

__global__ void Kernel(tTrackSeg* segArray, int nTrackSegs, State* graph,
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex, 
	int numPartialIterations, double randState,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime){

	curandState_t curandState;
	
	int idx = threadIdx.x + blockDim.x*blockIdx.x;
	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		idx, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);

	int offset = numPartialIterations*idx;

	int stateIterator = 0;

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

	for (stateIterator = offset; stateIterator < offset+numPartialIterations; stateIterator++){
		
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

		//the generation didnt work
		if (!ConstraintChecking::validPoint(segArray, nTrackSegs, &xRand, 0)){
			continue;
		}
		
		State* xNearest = EvalFunctions::nearestNeighbor(&xRand, graph, 10, actionSimDeltaTime); //GRAPH ITERATOR FUCKUP!
		xRand.setParent(xNearest);

		//--------------------------------------------------------------------------------------------------------------------------------

		DeltaHeuristics::applyDelta(&xRand, xNearest, segArray, nTrackSegs, forwardSegments, neighborDeltaPos, neighborDeltaSpeed);


		double cMin = xNearest->getPathCost() + EvalFunctions::evaluatePathCost(segArray, nTrackSegs, xNearest, &xRand, 30); //redifine path cost for new coords
		//double cMin = xNearest->getPathCost() + EvalFunctions::oldEvaluatePathCost( xNearest, xRand, this->forwardSegments);
		xRand.setPathCost(cMin);


		//the normalization didnt work
		if (!ConstraintChecking::validPoint(segArray, nTrackSegs, &xRand, -2.0)){
			continue;
		}

	}
	
	printf("executed Kernel!:%d\n",offset);

}


State* callKernel(tTrackSeg* segArray, int nTrackSegs,
	double minXVertex, double maxXVertex, double minYVertex, double maxYVertex,
	double numIterations,
	int forwardSegments, double neighborDeltaPos, double neighborDeltaSpeed, double actionSimDeltaTime){


	State* graph = new State[(unsigned int)numIterations];

	State* auxGraph;

	tTrackSeg* auxSegArray;

	int NUM_BLOCKS = 50;
	int NUM_THREADS_EACH_BLOCK = 10;
	int NUM_THREADS = NUM_BLOCKS*NUM_THREADS_EACH_BLOCK;

	int numPartialIterations = numIterations / NUM_THREADS;

	double size = sizeof(tTrackSeg)*(unsigned int)nTrackSegs;

	cudaMalloc(&auxGraph, sizeof(State)*(unsigned int)numIterations);
	cudaMalloc(&auxSegArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs);

	cudaMemcpy(auxGraph, graph, sizeof(State)*(unsigned int)numIterations, cudaMemcpyHostToDevice);
	cudaMemcpy(auxSegArray, segArray, sizeof(tTrackSeg)*(unsigned int)nTrackSegs, cudaMemcpyHostToDevice);
	Kernel << < NUM_BLOCKS, NUM_THREADS_EACH_BLOCK >> > (auxSegArray, nTrackSegs, auxGraph,
		minXVertex, maxXVertex, minYVertex, maxYVertex, 
		numPartialIterations, 0,
		forwardSegments,neighborDeltaPos,neighborDeltaSpeed,actionSimDeltaTime);
	cudaMemcpy(graph, auxGraph, sizeof(State)*(unsigned int)numIterations, cudaMemcpyDeviceToHost);
	cudaDeviceSynchronize();
	std::cout << "error1: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;

	cudaFree(auxGraph);
	cudaFree(auxSegArray);
	
	std::cout << "error2: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;
	


	return graph;
}
