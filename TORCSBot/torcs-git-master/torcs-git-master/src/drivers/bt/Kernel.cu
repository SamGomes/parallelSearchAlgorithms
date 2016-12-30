
#include "Kernel.cuh"


__global__ void kernel(State* initialState, State* returnedPath, int PATHMAXSIZE, double rand){
	curandState_t curandState;
	/* we have to initialize the state */
	curand_init(clock(), /* the seed controls the sequence of random values that are produced */
		0, /* the sequence number is only important with multiple cores */
		0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
		&curandState);

	//returnedPath[0] = State((curand_uniform(&curandState) * 2) - 1, (curand_uniform(&curandState) * 4 * 3.14f)  - 1.57f, initialState);
	printf("mthfk\n");
}

std::vector<State> cuda_search(State initialState){

	const int PATHMAXSIZE = 5;

	State* auxInitState;

	State* auxReturnedPath;
	
	State returnedPath[1];


	cudaMalloc(&auxInitState, sizeof(State));
	cudaMemcpy(auxInitState, &initialState, sizeof(State), cudaMemcpyHostToDevice);

	cudaMalloc(&auxReturnedPath, sizeof(State)*PATHMAXSIZE);
	cudaMemcpy(auxReturnedPath, &returnedPath, sizeof(State)*PATHMAXSIZE, cudaMemcpyHostToDevice);

	//kernel << < 64, 64 >> > (auxInitState, auxReturnedPath, PATHMAXSIZE);
	//srand(time(NULL));
	//double rand = (std::rand() / ((double)RAND_MAX / 2)) - 1;
	kernel << < 1, 1 >> > (auxInitState, auxReturnedPath, PATHMAXSIZE, 0);
	cudaMemcpy(&returnedPath, auxReturnedPath, sizeof(State)*PATHMAXSIZE, cudaMemcpyDeviceToHost);

	cudaFree(auxInitState);
	cudaFree(auxReturnedPath);

	/*cudaDeviceSynchronize();
	std::cout << "error: " << cudaGetErrorString(cudaPeekAtLastError()) << std::endl;
*/

	std::vector<State> ret = std::vector<State>(1);
	//ret.assign(returnedPath, returnedPath + sizeof(returnedPath)); //transform array in vector

	return ret;
}
