#ifndef CUDA_H
#define CUDA_H
#include "darknet.h"

#if defined(_MSC_VER) && _MSC_VER < 1900
	#define inline __inline
#endif

extern int gpu_index;

#ifdef GPU

#define BLOCK 512

#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

#ifdef CUDNN
#include "cudnn.h"
#endif // CUDNN

#ifndef __DATE__
#define __DATE__
#endif

#ifndef __TIME__
#define __TIME__
#endif

#ifndef __FUNCTION__
#define __FUNCTION__
#endif

#ifndef __LINE__
#define __LINE__ 0
#endif

#ifndef __FILE__
#define __FILE__
#endif

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus
    void check_error(cudaError_t status);
    void check_error_extended(cudaError_t status, char *file, int line, char *date_time);
#define CHECK_CUDA(X) check_error_extended(X, __FILE__ " : " __FUNCTION__, __LINE__,  __DATE__ " - " __TIME__ );

    cublasHandle_t blas_handle();
    float *cuda_make_array(float *x, size_t n);
    int *cuda_make_int_array(size_t n);
	int *cuda_make_int_array_new_api(int *x, size_t n);
    void cuda_push_array(float *x_gpu, float *x, size_t n);
    //LIB_API void cuda_pull_array(float *x_gpu, float *x, size_t n);
    //LIB_API void cuda_set_device(int n);
    int cuda_get_device();
    void cuda_free(float *x_gpu);
    void cuda_random(float *x_gpu, size_t n);
    float cuda_compare(float *x_gpu, float *x, size_t n, char *s);
    dim3 cuda_gridsize(size_t n);
    cudaStream_t get_cuda_stream();
    cudaStream_t get_cuda_memcpy_stream();
    int get_number_of_blocks(int array_size, int block_size);
#ifdef __cplusplus
}
#endif // __cplusplus

#ifdef CUDNN
cudnnHandle_t cudnn_handle();
enum {cudnn_fastest, cudnn_smallest};

void cudnn_check_error_extended(cudnnStatus_t status, char *file, int line, char *date_time);
#define CHECK_CUDNN(X) cudnn_check_error_extended(X, __FILE__ " : " __FUNCTION__, __LINE__,  __DATE__ " - " __TIME__ );
#endif

#else // GPU
//LIB_API void cuda_set_device(int n);
#endif // GPU
#endif // CUDA_H
