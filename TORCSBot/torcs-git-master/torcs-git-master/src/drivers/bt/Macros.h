#ifdef __CUDACC__
#define CUDA_HOSTDEV __host__ __device__ 
#define CUDA_DEV  __device__ 
#define CUDA_HOST __host__ 
#define CUDA_GLOBAL __global__
#else
#define CUDA_HOSTDEV
#define CUDA_DEV  
#define CUDA_HOST 
#define CUDA_GLOBAL
#endif