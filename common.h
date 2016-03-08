
#include<iostream>
#include<math.h>

#include <limits>               //定义无穷大用
#include <cuda_runtime_api.h>
#include <vector_functions.h>
using namespace std;

#include <cstring>
#include <cstdio>
#include<fstream>


#define PI 3.1415926535898 
#define eposi 0.001
#define INF  std::numeric_limits<float>::infinity()


#define NORMALIZE(A) {float l = 1.0f / sqrtf(A.x * A.x + A.y * A.y + A.z * A.z); A.x *= l; A.y *= l; A.z *= l;}
#define LENGTH(A)		(sqrtf(A.x*A.x+A.y*A.y+A.z*A.z))
#define CROSS(A,B)    (make_float3( A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x ))
#define DOT(A,B)  (A.x * B.x + A.y * B.y + A.z * B.z)
