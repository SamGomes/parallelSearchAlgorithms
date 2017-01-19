
#include "Kernel.cuh"


__global__ void kernel(State* initialState, State* returnedPath, int PATHMAXSIZE, double rand){
	curandState_t curandState;
	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		0, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);
	double randInc = (double)curand_uniform(&curandState) * (double) 50;

	tPosd randPos = { initialState->getPos().x + randInc
		, initialState->getPos().y + randInc
		, initialState->getPos().z + randInc };
	
	tPosd randSpeed = { initialState->getSpeed().x + randInc
		, initialState->getSpeed().y + randInc
		, initialState->getSpeed().z + randInc };

	returnedPath[0] = State(randPos,randSpeed,initialState->getAcceleration(),initialState);
	//printf("executed Kernel!\n");
}

State* cuda_search(State initialState){

	const int PATHMAXSIZE = 1;

	State* auxInitState;

	State* auxReturnedPath;
	
	State returnedPath[1];


	cudaMalloc(&auxInitState, sizeof(State));
	cudaMemcpy(auxInitState, &initialState, sizeof(State), cudaMemcpyHostToDevice);

	cudaMalloc(&auxReturnedPath, sizeof(State)*PATHMAXSIZE);
	cudaMemcpy(auxReturnedPath, &returnedPath, sizeof(State)*PATHMAXSIZE, cudaMemcpyHostToDevice);

	kernel << < 10, 2 >> > (auxInitState, auxReturnedPath, PATHMAXSIZE,0);
	
	//kernel << < 1, 1 >> > (auxInitState, auxReturnedPath, PATHMAXSIZE, 0);
	cudaMemcpy(&returnedPath, auxReturnedPath, sizeof(State)*PATHMAXSIZE, cudaMemcpyDeviceToHost);
	
	cudaDeviceSynchronize();
	
	cudaFree(auxInitState);
	cudaFree(auxReturnedPath);

	
	//std::cout << "error: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;
	

	return new State(*returnedPath);
}
