
#include"Interface.h"

#include<cuda.h>
#include<stdio.h>
#include <limits>               //定义无穷大用

enum
{
	SPHERETYPE = 1,
	PLANETYPE,
	CYLINDERTYPE,
	MESHTYPE,
	BOXTYPE
};

#define INF_CUDA  100000000000000
//基本操作
int* g_ScanSum[2];

__device__ float3 operator+(const float3 &p1, const float3 &p2)         //向量加
{
	return make_float3( p1.x+p2.x, p1.y+p2.y, p1.z+p2.z );
}

__device__ float3 operator-(const float3 &p1, const float3 &p2)           //向量减
{
	return make_float3( p1.x-p2.x, p1.y-p2.y, p1.z-p2.z );
}

__device__ float3 operator-(const float3 &p )           //向量减
{
	return make_float3(-p.x, -p.y, -p.z);
}

__device__ float operator*(const float3 &v1, const float3 &v2){         //向量乘
	return v1.x*v2.x+v1.y*v2.y+v1.z*v2.z;
}

__device__ float3 operator*(float x, const float3 &p)              //向量数乘
{
	float3 p1;
	p1.x=x*p.x,p1.y=x*p.y,p1.z=x*p.z;
	return p1;
}

__device__ float3 operator*( const float3 &p, float x)              //向量数乘
{
	float3 p1;
	p1.x=x*p.x,p1.y=x*p.y,p1.z=x*p.z;
	return p1;
}

__device__ float3 cross(const float3 &v1, const float3 &v2){            //向量叉积
	float3 vout;
	vout.x=v1.y*v2.z-v1.z*v2.y;
	vout.y=v1.z*v2.x-v1.x*v2.z;
	vout.z=v1.x*v2.y-v2.x*v1.y;
	return vout;
}

__device__ float moduleSqr(const float3 &v)                //向量的模的平方
{
	return v.x * v.x + v.y * v.y + v.z * v.z ;
}

__device__ float3 Vec_Normlize(const float3 &v)             //向量单位化
{
	float3 vout;
	float Module = sqrtf( moduleSqr( v ) ); 
	vout.x = v.x * (1.0 / Module);
	vout.y = v.y * (1.0 / Module);
	vout.z = v.z * (1.0 / Module);
	return vout;
}

__device__ float3 Color_Dot( float3 Col_left,float3 Col_Right)
{
	return make_float3(Col_left.x * Col_Right.x , Col_left.y * Col_Right.y ,Col_left.z * Col_Right.z );
}

__device__ float3 Color_Clamp( float3 m_Color )
{
	if( m_Color.x > 1.0 ) m_Color.x = 1.0;
	if( m_Color.y > 1.0 ) m_Color.y = 1.0;
	if( m_Color.z > 1.0 ) m_Color.z = 1.0;
	if( m_Color.x < 0 ) m_Color.x = 0;
	if( m_Color.y < 0 ) m_Color.y = 0;
	if( m_Color.z < 0 ) m_Color.z = 0;
	return m_Color;
}

//__constant__ Vector3D *LightPot_dev;      //点光源

//__constant__ Vector3D *LightDir_dev;          //点光源方向

//__constant__ Vector3D *Ia_dev ;                //环境光

//__device__ LightRay *Lr_dev;                      //跟踪光线

//__constant__ Sphere *Sp_dev;                        //测试球面

//__constant__ Cylinder *Cylind_dev;             //圆柱


//reflect direction
__device__ float3 d_reflect( const float3& dir , const float3& normal )
{
	float dotProduct = ( -2.0f ) * dir * normal;

	float3 r = dir + dotProduct * normal;

	return make_float3( r.x , r.y , r.z );
}

//refraction direction
__device__ float3 d_refract( const float3& dir , float3 normal , float rate )     
{
	float3 r;

	if(  dir * normal  > 0 )          //表示光线从物体中出来进入空气
	{
		normal = -1.0 * normal;
		rate = 1.0f / rate;
	}

	float cos = -1.0f * dir * normal;
	float t = 1 - rate * rate * ( 1 - cos * cos );

	if( t < 0 )
	{
		r = d_reflect( dir , normal );    //全反射
	}else
	if(t > 0)
	{
		float cos2 = sqrt( t );
		r = rate * dir + ( rate * cos - cos2 ) * normal ;
	}

	return r;
}

__device__ int Ray_Circle_Intersection( LightRay &L, float3 &Center, float radiu, float *t1, float *t2 )       //光线与圆求交
{
	//光线与圆相交的方程： a * t^2 + b * t + c = 0
	//a = Ray.Direction * Ray.Direction = 1;  b = 2 * Ray.Direction * (Ray.Origin - Center); c = ||Ray.Origin - Center||^2 - Radius^2

	float3 OC = L.Orig -Center;
	float a,b;                      //对应将光线代入平面圆方程得到的方程 a*t^2+b*t+c=0;
	a = L.Dir * L.Dir;
	b = 2.0 * L.Dir * OC;
	
	float delta = b * b - 4 * a * (OC * OC - radiu * radiu);
	if(delta > 0)
		return 1;
	else return 0;
}

__device__ int Ray_Cylinder_Intersection(const LightRay& L,const Cylinder& Cylid, float& t_min, float3 &Out_Norm) //判断光线和底面平行于z=0的柱面相交
{
	float a,b,c;
	float x_e, y_e;
	a = L.Dir.x * L.Dir.x + L.Dir.y * L.Dir.y;  // a=x_d^2 + y_d^2
	x_e = L.Orig.x - Cylid.center.x, y_e = L.Orig.y - Cylid.center.y;
	b = 2.0 *(x_e * L.Dir.x + y_e * L.Dir.y);
	c = x_e * x_e + y_e * y_e - Cylid.radiu * Cylid.radiu ;
	float delta = b * b - 4.0 * a * c;
	if(delta > 0)
	{
		float t1 = (-b + sqrt(delta))/(2.0 * a);  //大
		float t2 = (-b - sqrt(delta))/(2.0 * a);    //小
		float t;
		float z_min = Cylid.center.z, z_max = Cylid.center.z + Cylid.height;
		float z1 = L.Orig.z + t1 * L.Dir.z;
		float z2 = L.Orig.z + t2 * L.Dir.z;   

		if(t1>0 )
		{
			if(t2>0)                        //视点在圆柱外的情况
			{
				
				if( z2 > z_max && z1 < z_max ) //交于上底面  
				{
					t =( z_max - L.Orig.z )/L.Dir.z;    //因为交于上底面，所以z分量不为0
					if( t < t_min)
					{
						t_min = t;
						Out_Norm = make_float3(0,0,1);
						return 1;
					}					
				}
				else if(z2 < z_max && z2 > z_min) //交于侧面
				{
					if(t2 < t_min)
					{
						t_min = t2;
						Out_Norm =Vec_Normlize( L.Orig + t_min * L.Dir - make_float3(Cylid.center.x,Cylid.center.y,z2) );
						return 1;
					}
					
				}

				else if( z2 < z_min  && z1 > z_min  ) //交于下底面  
				{
					t =( z_min - L.Orig.z )/L.Dir.z;
					if( t< t_min)
					{
						t_min = t;					
						Out_Norm = make_float3(0,0,-1);
						return 1;
					}
			
				}

			}
			else  //t2<0      
			{
				if( z2 < z_min  && z1 > z_min  ) //交于下底面  
				{
					t =( z_min - L.Orig.z )/L.Dir.z;
					if( t< t_min)
					{
						t_min = t;					
						Out_Norm = make_float3(0,0,-1);
						return 1;
					}

				}
				else if( z2 > z_max && z1 < z_max)   //交与上底面
				{
					t =( z_max - L.Orig.z )/L.Dir.z;
					if( t< t_min)
					{
						t_min = t;					
						Out_Norm = make_float3(0,0,1);
						return 1;
					}


				}

			}
	
		}
		
		
	}



	return 0;

}

__device__ float3 GetBoxNormal(const float3& a_Pos, const Box& m_Box)
{
	if (a_Pos.x - m_Box.pos.x < eposi)
	{
		return make_float3(-1, 0, 0);
	}
	if (a_Pos.y - m_Box.pos.y < eposi)
	{
		return make_float3(0, -1, 0);
	}
	if (a_Pos.z - m_Box.pos.z < eposi)
	{
		return make_float3(0, 0, -1);
	}
	if ((m_Box.pos + m_Box.size).x - a_Pos.x < eposi)
	{
		return make_float3(1, 0, 0);
	}
	if ((m_Box.pos + m_Box.size).y - a_Pos.y < eposi)
	{
		return make_float3(0, 1, 0);
	}
	if ((m_Box.pos + m_Box.size).z - a_Pos.z < eposi)
	{
		return make_float3(0, 0, 1);
	}

	return make_float3(0, 0, 0);
}


//check if the ray intersects with bounding box
__device__ float3 kernelIntersectBoundingBox( const float3& ori , const float3& dir , const float3& min , const float3& max , const float length ) //result.x记录最近交点，result.z标识是否有交
{
	//the result
	float3 result = make_float3( 0.0f , 9999999.0f , 0.0f ); //result.x记录最近交点，result.z标识是否有交

	//limit the maxium value
	if( length > 0 )
		result.y = length;

	//the variables
	float t1 , t2;

	if( fabs( dir.x ) < 0.0000001f )
	{
		if( ori.x > max.x || ori.x < min.x )
			return result;
	}else
	{
		t1 = ( max.x - ori.x ) / dir.x;
		t2 = ( min.x - ori.x ) / dir.x;

		if( t1 > t2 ) { float t = t1; t1 = t2; t2 = t; }    //t1永远最小

		//clamp
		if( t1 > result.x ) result.x = t1;
		if( t2 < result.y ) result.y = t2;

		if( result.x > result.y )               //不相交
			return result;
	}

	if( fabs( dir.y ) < 0.0000001f )
	{
		if( ori.y > max.y || ori.y < min.y )
			return result;
	}else
	{
		t1 = ( max.y - ori.y ) / dir.y;
		t2 = ( min.y - ori.y ) / dir.y;

		if( t1 > t2 ) { float t = t1; t1 = t2; t2 = t; }

		//clamp
		if( t1 > result.x ) result.x = t1;
		if( t2 < result.y ) result.y = t2;

		if( result.x > result.y )
			return result;
	}

	if( fabs( dir.z ) < 0.0000001f )
	{
		if( ori.z > max.z || ori.z < min.z )
			return result;
	}else
	{
		t1 = ( max.z - ori.z ) / dir.z;
		t2 = ( min.z - ori.z ) / dir.z;

		if( t1 > t2 ) { float t = t1; t1 = t2; t2 = t; }

		//clamp
		if( t1 > result.x ) result.x = t1;
		if( t2 < result.y ) result.y = t2;

		if( result.x > result.y )
			return result;
	}

	//标识有交
	result.z = 1.0f;

	return result;
}

//do interplotation
__device__ float3 kernelInterploted( const float3& v1 , const float3& v2 , const float3& v3 , const float3& intersected )
{
	//get the vectors
	float3 e1 = intersected - v1;
	float3 e2 = intersected - v2;
	float3 e3 = intersected - v3;

	//compute the areas
	float3 area;
	area.x = LENGTH( cross( e2 , e3 ) );
	area.y = LENGTH( cross( e3 , e1 ) );
	area.z = LENGTH( cross( e1 , e2 ) );

	float d = 1.0f / ( area.x + area.y + area.z );

	return area * d;
}


//check if the ray intersects with a plane
__device__ int kernelIntersectTriPlane( const float3& v1 , const float3& v2 , const float3& v3 , const float3& ori , const float3& dir, float3& Hit,  float& t)
{
// 
// 	float d = L.Dir * Plan_dev.norm;
// 	if(d < 0 )
// 	{
// 		float t_min = -( Plan_dev.norm * L.Orig + Plan_dev.D )/d;
// 		if(t_min < t )
// 		{
// 			t = t_min;
// 			norm = Plan_dev.norm;
// 			return 1;
// 		}
// 	}
// 	return 0;

	//w : >= 0 ( intersected point enable ) , < 0 ( disable )
	Hit = make_float3( 0.0f , 0.0f , 0.0f );

	//get the normal of the plane
	float3 normal = cross( v2 - v1 , v3 - v1 );

	//get the factor
	t =  normal * ( ori - v1 ) / ( normal * dir );

	//set the result
	Hit = ori - t * dir;

	if( t <= 0.0f ) //跟平面有交
		return 1;
	else
		return -1;
}

//check if the ray intersects with a triangle
__device__ int kernelIntersectTriangle( const Triangle_mesh& Tri_mesh, const float3& ori , const float3& dir, float3& hit_norm, float& t_min)
{

	float3 v1,v2,v3,n1,n2,n3;
	v1 = Tri_mesh.pt1,v2 = Tri_mesh.pt2, v3 = Tri_mesh.pt3;
	n1 = Tri_mesh.N1, n2 = Tri_mesh.N2, n3 = Tri_mesh.N3;
	float t;
	float3 hit_pt;
	int res = kernelIntersectTriPlane( v1 , v2 , v3 , ori , dir, hit_pt, t );

	if( res < 0 )
		return 0;

	//get the factor
	float3 d1 = cross( hit_pt - v2 , v1 - v2 );
	float3 d2 = cross( hit_pt - v3 , v2 - v3 );
	float3 d3 = cross( hit_pt - v1 , v3 - v1 );

	float f1 =  d1 * d2;
	float f2 =  d2 * d3;

	if( !( f1 >= -0.000000000000001f && f2 >= -0.000000000000001f ) ) //无交
		return 0;
    if( -t < t_min )
	{
		t_min = -t;
//  		float3 interp = kernelInterploted(v1, v2, v3, hit_pt);
//  		hit_norm = n1 * interp.x + n2 * interp.y + n3 * interp.z;
		hit_norm = Tri_mesh.Face_Nrm;
		return 1;
	}

	return 0;
}

__device__ int Ray_Sphere_Intersect(const LightRay &L,const Sphere &S, float& t, float3 &norm)    //判断并计算光线与球面的交，若相交，t1返回最近的交点参数值
                                                                       //包含光源位于球面内的情况
{
	//光线与球相交的方程： a * t^2 + b * t + c = 0
	//a = Ray.Direction * Ray.Direction = 1;  b = 2 * Ray.Direction * (Ray.Origin - Center); c = ||Ray.Origin - Center||^2 - Radius^2
	float3 OC = L.Orig -S.center;
	//delta = b * b - 4 * a * c
	float a = L.Dir * L.Dir;
	float b = 2 * (L.Dir * OC);
	float delta = b * b - 4 * a * (OC * OC - S.radiu * S.radiu);
	if (delta > 0)
	{
		delta = sqrtf(delta);
		//求出两个根，小的即是距离
		float i1 = (- b + delta )/(2.0 * a), i2 =( - b - delta )/(2.0 * a);
		if (i1 > 0)
		{
			//视点在球内部
			if (i2 < 0)
			{
				if (i1 < t)
				{
					t = i1;
					norm = Vec_Normlize(( L.Orig + t * L.Dir ) - S.center ); //内外法向要统一
					return 1;
				}
			}
			//视点在球外
			else
			{
				if (i2 < t)
				{
					t = i2;
					norm = Vec_Normlize( L.Orig + t * L.Dir - S.center );
					return 2;
				}
			}
		}
	}
	return 0;
}


__device__ int Ray_Plane_Intersect(const LightRay &L, const plane& Plan_dev, float &t, float3& norm )
{
	float d = L.Dir * Plan_dev.norm;
	if(d < 0 )
	{
		float t_min = -( Plan_dev.norm * L.Orig + Plan_dev.D )/d;
		if(t_min < t )
		{
			t = t_min;
			norm = Plan_dev.norm;
			return 1;
		}
	}
	return 0;

}

// 
// //the classic ray triangle intersection: http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
// __device__ int Ray_Triangle_Intersection(const LightRay &r, 
// 										   const float3 &v1, 
// 										   const float3 &v2, 
// 										   const float3 &v3, const float3& n1, const float3& n2,const float3& n3, float3 &hit_pt, float3& hit_norm, float& t_min )
// {  
// 	float3 edge1 = v2 - v1;
// 	float3 edge2 = v3 - v1;
// 
// 	float3 vg1 = r.Orig - v1;  
// 	float3 vg2 = cross(r.Dir, edge2);      
// 	float  det  = edge1 * vg2;  
// 
// 	//	det = __fdividef(1.0f, det);  
// 	if( det)
//     det = 1.0/det;
// 
// 	float u = vg1 * vg2 * det;  
// 
// 	if (u < 0.0f || u > 1.0f)  
// 		return 0;  
// 
// 	float3 vg3 = cross(vg1, edge1);  
// 
// 	float vg = r.Dir * vg3 * det;  
// 
// 	if (vg < 0.0f || (u + vg) > 1.0f)  
// 		return 0;  
// 
// 	float t = edge2 * vg3 * det;
// 	if( t < t_min )
// 	{
// 		t_min = t;
// 		hit_pt = r.Orig + t_min * r.Dir;
// 		float3 interp = kernelInterploted(v1, v2, v3, hit_pt);
// //		hit_norm = n1 * interp.x + n2 * interp.y + n3 * interp.z;
// 		return 1;
// 	}
// 	return 0;
// }  


void checkCudaError()
{
    cudaError_t err = cudaGetLastError();

    if (cudaSuccess != err)
    {
        fprintf(stderr, "Cuda error: %s.\n",
                cudaGetErrorString(err));
    }
}

__global__ void kernelInitBuffer( float3* img_buffer, int* mark , int pixelNum )
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= pixelNum )
		return;
	
	img_buffer[tid] =make_float3(0.0, 0.0, 0.0);
	mark[tid] = tid;
}

extern "C" void cudaInitBuffer( float3* img_buffer, int* mark , int pixelNum )
{
	//the block number
	int threads = 256;
	int blocks = ( pixelNum + threads - 1 ) / threads;

	//call the kenrel
	kernelInitBuffer<<<blocks,threads>>>( img_buffer , mark , pixelNum );
}


__global__ void kernelGeneratePrimaryRays( const float3* eye,const float3* LookAt, const int w ,const int h, LightRay *dev_ray)
{
	float3 view = make_float3(0,0,0);      //视点坐标系下的视点坐标

	float3 m_Win1 = make_float3(-5,-5,5);
	float3 m_Win2 = make_float3(5,-5, 5);
	float3 m_Win3 = make_float3(5,5, 5);
	float3 m_Win4 = make_float3(-5, 5, 5);
	
// 	float3 m_Win1 = make_float3(-4,-3,0);
// 	float3 m_Win2 = make_float3(4,-3, 0);
// 	float3 m_Win3 = make_float3(4,3, 0);
// 	float3 m_Win4 = make_float3(-4, 3, 0);

	float3 cZaxis = Vec_Normlize( *LookAt - *eye );
	
	float3 up = make_float3(0,0,1);
// 	if(cZaxis.x > 0)
// 		up = make_float3(0,0, -1 );
/*	float3 up1 = make_float3(0,0,1);*/

	float3 cYaxis = cross( cZaxis, up  );
// 	if( moduleSqr(cYaxis) < eposi )
// 		cYaxis = cross( up1, cZaxis);
	float3 cXaxis = cross(cYaxis  , cZaxis);    //场景坐标系下的坐标

	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= w * h )
		return;

	// get the pixel coorindate first
	uint2 coord;
	coord.y = tid % (int) w;
	coord.x = tid / (int) w;  

	float3 v =Vec_Normlize( m_Win1 + coord.x * (1.0/w) * (m_Win2 -m_Win1) + coord.y * (1.0/h) * (m_Win4 - m_Win1)  - view );   //首先得到视域空间下的光线方向


	float3 invView1 = make_float3(cXaxis.x,cYaxis.x,cZaxis.x);
	float3 invView2 = make_float3(cXaxis.y,cYaxis.y,cZaxis.y);
	float3 invView3 = make_float3(cXaxis.z,cYaxis.z,cZaxis.z);

	v = make_float3(v * invView1 ,  v * invView2  , v * invView3  );


	view = make_float3(view * invView1 + eye->x, view * invView2 + eye->y, view * invView3 + eye->z);

	 dev_ray[tid].Orig= make_float3( view.x, view.y, view.z);
	 dev_ray[tid].Dir = make_float3(v.x, v.y, v.z);
	 dev_ray[tid].order = tid;
	 dev_ray[tid].intensity = 1.0;

}

extern "C" void cudaGeneratePrimaryRays( const float3* eye, const float3* LookAt, const int w ,const int h, LightRay *dev_ray)

{
	//get the number of data
	int rayNum =  w * h ;

	//the block number
	int threadNum = 64;
	int blockNum = ( rayNum + threadNum - 1 ) / threadNum;

	//call the kernel
	kernelGeneratePrimaryRays<<<blockNum , threadNum>>>( eye , LookAt, w, h, dev_ray );
}

__device__ float3 GetTexel(const float& a_U, const float& a_V, const Texture& m_texture)   //a_u,a_v在0,1之间
{
	int m_Width = m_texture.m_Width, m_Height = m_texture.m_Height;
	float fu = fabsf(a_U) * m_Width, fv = fabsf(a_V) * m_Width;
	int u1 = ((int)fu) % m_Width, v1 = ((int)fv) % m_Height;
	int u2 = (u1 + 1) % m_Width, v2 = (v1 + 1) % m_Height;

	//双线性插值
	float uFrac = fu - floorf(fu), vFrac = fv - floorf(fv);
	float w1 = (1 - uFrac) * (1 - vFrac), 
		w2 = uFrac * (1 - vFrac),
		w3 = (1 - uFrac) * vFrac,
		w4 = uFrac * vFrac;

	//取四个点的像素值
	int size = m_Width * m_Height;
	int x1 = (u1 + v1 * m_Width) % size,
		x2 = (u2 + v1 * m_Width) % size,
		x3 = (u1 + v2 * m_Width) % size,
		x4 = (u2 + v2 * m_Width) % size;
	x1 = x1 < 0 ? 0 : x1;
	x2 = x2 < 0 ? 0 : x2;
	x3 = x3 < 0 ? 0 : x3;
	x4 = x4 < 0 ? 0 : x4;
	float3 c1 = m_texture.cuda_Bitmap[x1],
		c2 = m_texture.cuda_Bitmap[x2],
		c3 = m_texture.cuda_Bitmap[x3],
		c4 = m_texture.cuda_Bitmap[x4];
	return c1 * w1 + c2 * w2 + c3 * w3 + c4 * w4;
}

__device__ float3 CalcuPlanTexture(const plane& m_plan, const float3& hit_pt, const Texture* m_texture)    //计算平面纹理坐标
{
	float3 m_UAxis = m_plan.m_UAxis, m_VAxis = m_plan.m_VAxis;
	float u = (hit_pt * m_UAxis) * m_plan.m_UScale,
		  v = (hit_pt * m_VAxis) * m_plan.m_VScale;
	int tex_index = m_plan.tex_index;
	return Color_Dot( GetTexel(u,v,m_texture[tex_index]) , m_plan.color );
}

__device__ float3 CalcuCylinderTexture(const Cylinder& m_Cylin, const float3& hit_Norm, const float3& hit_pt, const Texture* m_texture )
{
// 	int tex_index = m_Cylin.tex_index;
// 	float Tex_w = m_texture[tex_index].m_Width, Tex_h = m_texture[tex_index].m_Height;
// 
// //	float3 m_UAxis = m_Cylin.m_UAxis, m_VAxis = m_Cylin.m_VAxis;
// 	float u = ( ( 1.0 + hit_Norm.x ) / 4.0 * Tex_w ) * m_Cylin.m_UScale,
// 		  v = ( ( hit_pt.z ) / m_Cylin.height * Tex_h ) * m_Cylin.m_VScale;
//     return GetTexel(u,v,m_texture[tex_index]);

	int tex_index = m_Cylin.tex_index;
	float Tex_w = m_texture[tex_index].m_Width, Tex_h = m_texture[tex_index].m_Height;
	//计算纹理坐标
	float u = acosf((hit_pt.x - m_Cylin.center.x)/m_Cylin.radiu)/(2.0f * PI) * m_Cylin.m_UScale;
	float v =  (hit_pt.z - m_Cylin.center.z)/m_Cylin.height * m_Cylin.m_VScale;

// 	float3 m_UAxis = make_float3(hit_Norm.y, hit_Norm.x, hit_Norm.z);
// 	float3 m_VAxis = CROSS( hit_Norm, m_UAxis );
// 
// 	float u = (hit_pt * m_UAxis) * m_Cylin.m_UScale,
// 		v = (hit_pt * m_VAxis) * m_Cylin.m_VScale;

	return Color_Dot( GetTexel(u,v,m_texture[tex_index]), m_Cylin.color );

}

__device__ float3 CalcuSphereTexture(const Sphere& Sp,  const float3& hit_pt, const Texture* m_texture)
{
	int index = Sp.tex_index;
	float3 Vecp = (hit_pt - Sp.center) *(1.0f / Sp.radiu);
	float3 zAxis = make_float3( 0, 0, 1 );
	float r = acosf(  Vecp * zAxis );  // 0-PI
	//	float u, v = r / (Sp.m_VScale * PI);
	float u, v = r /  PI * Sp.m_VScale;
	float theta = acosf( DOT( Vecp, make_float3(1, 0, 0) ) ) / (sinf(r) * 2.0f * PI);
	if (DOT(Vecp, make_float3(0, 1, 0 )) <= 0)
	{
		u = (1 - theta) * Sp.m_UScale;
	}
	else
	{
		u = theta * Sp.m_UScale;
	}

	return Color_Dot(GetTexel(u,v,m_texture[index]), Sp.color);
}

__device__ float3 CalcuBoxTexture(const Box& dev_Box,const float3& hit_pt, const Texture* m_texture)
{
	float3 vec = hit_pt - dev_Box.pos;
	float u_size = dev_Box.size * dev_Box.U_axis,
		  v_size = dev_Box.size * dev_Box.V_axis;

	int tex_index = dev_Box.tex_index;


	float u = vec * dev_Box.U_axis/u_size * dev_Box.u_scale,
		v = vec * dev_Box.V_axis/v_size * dev_Box.v_scale;

	return Color_Dot( GetTexel(u,v,m_texture[tex_index]) , dev_Box.color );
}

__device__ float kernelTraverseRay( const LightRay& ray, float3& intersect, float3& norm, float3& m_Color, float3& illums, float3& materialInf,const Sphere* Sp_dev,const plane* Plan_dev,Cylinder* Cylin_dev, const Texture* tex_dev, Mesh* Mesh_dev , Box* Box_dev, float& m_tmin, bool shadowTest )
{
	int i,j, instance = -1;
	float shadow = 1.0;     //阴影强度初始化
	float t_min = m_tmin;
	bool isplane = false;

	float3 result;

	int hitType;

	for(i=0;i<3;i++)
	{
		result = kernelIntersectBoundingBox(ray.Orig, ray.Dir, Sp_dev[i].min_pt, Sp_dev[i].max_pt, t_min); //先检验是否与包围盒有交  这里t_min不改变
		if( result.z > 0.5 )
		{
			if(Ray_Sphere_Intersect( ray, Sp_dev[i], t_min, norm ))
			{
				shadow *= Sp_dev[i].transparency;
				instance = i;
				hitType = SPHERETYPE;
			}
		}
	}
		
	for( i=0; i<6; i++)
	{
		if(Ray_Plane_Intersect(ray, Plan_dev[i], t_min, norm ))
		{
			shadow *= Plan_dev[i].transparency;
			instance = i;
			hitType = PLANETYPE;
		}
	}

	for( i=0; i<4; i++)
	{
		result = kernelIntersectBoundingBox(ray.Orig, ray.Dir, Cylin_dev[i].min_pt, Cylin_dev[i].max_pt, t_min); //先检验是否与包围盒有交  这里t_min不改变
		if(result.z > 0.5)
			if(Ray_Cylinder_Intersection(ray, Cylin_dev[i], t_min, norm ))
			{
				shadow *= Cylin_dev[i].transparency;
				instance = i;
				hitType = CYLINDERTYPE;
			}
	}

	for( i=0; i<1; i++)
	{
		result = kernelIntersectBoundingBox(ray.Orig, ray.Dir, Box_dev[i].pos, Box_dev[i].pos + Box_dev[i].size, t_min); //先检验是否与包围盒有交  这里t_min不改变	
		if(result.z > 0.5)
		{
			t_min = result.x;
			shadow *= Box_dev[i].transparent;
			instance = i;
			hitType = BOXTYPE;
		}
	}

 	for( i=0; i<1; i++)
 	{ 
		//kernelIntersectBoundingBox( float3& ori , float3& dir , float3& min , float3& max , float length )
	//	float t_temp = INF_CUDA;
 		result = kernelIntersectBoundingBox(ray.Orig, ray.Dir, Mesh_dev[i].min_pt, Mesh_dev[i].max_pt, t_min); //先检验是否与包围盒有交  这里t_min不改变
 		if( result.z > 0.5 )      //有交
		{
			for(j=0; j< Mesh_dev[i].patch_num; j++)
				if( kernelIntersectTriangle( Mesh_dev[i].cuda_Trimesh[j], ray.Orig,ray.Dir, norm, t_min ))
				{
					shadow *= Mesh_dev[i].transparent;
					instance = i;
					hitType = MESHTYPE;
				}
// 			{
// 				float3 p1,p2,p3, n1,n2,n3;
// 				p1 = Mesh_dev[i].cuda_Trimesh[j].pt1, p2 = Mesh_dev[i].cuda_Trimesh[j].pt2, p3 = Mesh_dev[i].cuda_Trimesh[j].pt3;
// 				n1 = Mesh_dev[i].cuda_Trimesh[j].N1, n2 = Mesh_dev[i].cuda_Trimesh[j].N2, n3 = Mesh_dev[i].cuda_Trimesh[j].N3;
// 				if( Ray_Triangle_Intersection(ray, p1, p2, p3 , n1, n2, n3, intersect, norm, t_min ))
// 				{
// 					norm = Mesh_dev[i].cuda_Trimesh[j].Face_Nrm;
// 					shadow *= Mesh_dev[i].transparent;
// 					instance = i;
// 					hitType = MESHTYPE;
// 				}
// 			}
		}
	}
	 

    if(shadowTest)
		return shadow;    

	if(instance > -1)
	{
		switch( hitType )
		{

		case PLANETYPE:
			{			
				intersect = ray.Orig + t_min * ray.Dir;
				illums = make_float3(Plan_dev[instance].kd,  Plan_dev[instance].ks,  1.0);
				materialInf = make_float3(Plan_dev[instance].refl, Plan_dev[instance].refr, Plan_dev[instance].transparency);
				if(Plan_dev[instance].tex_index < -0.2)   //没纹理
					m_Color = Plan_dev[instance].color;	 
				else
					m_Color = CalcuPlanTexture(Plan_dev[instance], intersect, tex_dev );
				break;
			}
		case SPHERETYPE:
			{
				intersect = ray.Orig + t_min * ray.Dir;
				illums = make_float3(Sp_dev[instance].kd,  Sp_dev[instance].ks,  1.0);
				materialInf = make_float3(Sp_dev[instance].refl, Sp_dev[instance].refr, Sp_dev[instance].transparency);
				if(Sp_dev[instance].tex_index < -0.2)   //没纹理
					m_Color = Sp_dev[instance].color;	 
				else
					m_Color = CalcuSphereTexture(Sp_dev[instance], intersect, tex_dev);	
				break;
			}
		case CYLINDERTYPE:
			{
				intersect = ray.Orig + t_min * ray.Dir;
				illums = make_float3(Cylin_dev[instance].kd,  Cylin_dev[instance].ks,  1.0);
				materialInf = make_float3(0.0, 0.0, Cylin_dev[instance].transparency);
				
				if(Cylin_dev[instance].tex_index < -0.2)
					m_Color = Cylin_dev[instance].color;	
				else
					m_Color = CalcuCylinderTexture(Cylin_dev[instance], norm, intersect, tex_dev );   //
				break;
			}
		case MESHTYPE:
			{
				intersect = ray.Orig + t_min * ray.Dir;
				illums = make_float3(Mesh_dev[instance].kd, Mesh_dev[instance].ks, 1.0 );
				materialInf = make_float3(0.0, 0.0, Mesh_dev[instance].transparent);
				m_Color = Mesh_dev[instance].color;
				break;
			}
		case BOXTYPE:
			{
				intersect = ray.Orig + t_min * ray.Dir;
				norm = GetBoxNormal(intersect, Box_dev[instance]);
				illums = make_float3(Box_dev[instance].kd, Box_dev[instance].ks, 1.0 );
				materialInf = make_float3(Box_dev[instance].refl, Box_dev[instance].refr, Box_dev[instance].transparent);
				if(Box_dev[instance].tex_index < -0.2)
					m_Color = Box_dev[instance].color;
				else
					m_Color = CalcuBoxTexture(Box_dev[instance], intersect, tex_dev);
				break;
			}
		default:
			break;
		}
	}
	return shadow;
}



__global__ void kernelGetIntersect(	LightRay*	rayOri , 
							 int ray_num,
							 float3* intersect,  
							 float3* m_Color,
							 float3* norm,
							 float3* illums,             //保存kd,ks
							 float3 *materialInfo,       //物体反射率，折射率，物体透明度
							 Texture* tex_dev,
							 Sphere* Sp_dev, plane* Plan_dev, Cylinder* Cylin_dev,Mesh* Mesh_dev, Box* Box_dev )
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= ray_num )
		return;

	float t_min = INF_CUDA;

	float shadowV = kernelTraverseRay(rayOri[tid], intersect[tid], norm[tid], m_Color[tid], illums[tid], materialInfo[tid], Sp_dev, Plan_dev, Cylin_dev, tex_dev, Mesh_dev, Box_dev, t_min, false);
	
}

extern "C" void cudaGetIntersect(LightRay*	rayOri , 
								int ray_num,					
								float3* intersect,
								float3* m_Color,
								float3* norm,
								float3* illums,
								float3 *materialInfo,     //物体反射率，折射率
								Texture* tex_dev,
								Sphere* Sp_dev, plane* Plan_dev, Cylinder* Cylin_dev,Mesh* Mesh_dev, Box* Box_dev ) 
									
{
// 	cudaEvent_t  start,stop;
// 	cudaEventCreate(&start);
// 	cudaEventCreate(&stop);
// 	cudaEventRecord(start,0); 
	//the block and thread number
	int threadNum = 64;
	int blockNum = ( ray_num + threadNum - 1 ) / threadNum ;

	//call the kernel
	kernelGetIntersect<<<blockNum , threadNum>>>(rayOri, ray_num, intersect, m_Color, norm, illums, materialInfo,tex_dev, Sp_dev, Plan_dev, Cylin_dev, Mesh_dev, Box_dev);

	cudaError_t error = cudaSuccess;
	error = cudaGetLastError();

	if (error != cudaSuccess)
	{
		printf("cudaGetIntersect() failed to launch error = %d, 意思是: %s\n", error, cudaGetErrorString(error));
	}

// 	cudaEventRecord(stop,0);
// 	cudaEventSynchronize(stop);
// 	float elapsedTime;
// 	cudaEventElapsedTime(&elapsedTime,start,stop);
// 	printf("Time to generate: %3.1f ms\n",elapsedTime);
}

__device__ void Shade( const float3& intersect,const float3& m_Color, const float3& Norm, const float& kd, const float& ks, 
					  const LightRay& Lr_dev, const lightpot* light, float3 &localColor, Sphere* Sp_dev, plane* Plan_dev,Cylinder* Cylin_dev, Texture* tex_dev,Mesh* Mesh_dev,  Box* Box_dev)                          //着色
{
	LightRay ShadowRay;
	float3 Hit = intersect;
	
	for(int i=0; i<3; i++)        //三个点光源
	{
		float3 L_Dir = light[i].Pos - Hit ;   

 		float tmpLength = sqrtf( moduleSqr(L_Dir) );
 		L_Dir = L_Dir * ( 1.0f / tmpLength) ;	     //交点到光源的单位向量
		
		ShadowRay.Orig = Hit + eposi * L_Dir;
		ShadowRay.Dir = L_Dir;
		ShadowRay.intensity = 1.0;

		float Shadow;
		float t_min = tmpLength;  //到光源的距离

		float3 a_norm, a_Color, a_illums,  a_materialInf, hitpt;

		Shadow = kernelTraverseRay(ShadowRay, hitpt, a_norm, a_Color,a_illums,a_materialInf, Sp_dev, Plan_dev,Cylin_dev, tex_dev,Mesh_dev, Box_dev, t_min, true);   //处在阴影区时值是0

		//phone 模型着色
		float  amt_NL;

		localColor = localColor + 0.2 * Color_Dot(0.3 * light[i].bright,m_Color );      //环境光，ka=0.2

		if(kd > 0)
		{
			amt_NL =  Norm * L_Dir; 
			if(amt_NL > 0)
				localColor = localColor + (kd * amt_NL * Color_Dot(light[i].bright, m_Color) * Shadow);     //漫射分量
		}


		if(ks > 0) //高光
		{
			float3 V = Lr_dev.Dir, R = L_Dir - Norm * 2.0f * ( Norm * L_Dir );
			float dot = V * R;
			if (dot > 0)
			{
				localColor = localColor + (light[i].bright * ks * powf(dot, 20) * Shadow);
			}
		}

	}
}

__global__ void kernelPixelShader(float3* img_buffer, int ray_num, float3* intersect, float3* Color, float3* norm, 
								  lightpot* Light_dev,float3* illums, int* mark, Sphere* Sp_dev, plane* Plan_dev,Cylinder* Cylin_dev,Texture* tex_dev,Mesh* Mesh_dev,  Box* Box_dev, LightRay* ray )
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= ray_num )
		return;

	float3 addColor= make_float3(0.0, 0.0, 0.0);

//	if(intersect[tid].w < 1.0 )       //have an intersect visible to the viewer
	Shade(intersect[tid],Color[tid], norm[tid], illums[tid].x, illums[tid].y, ray[tid], Light_dev, addColor, Sp_dev, Plan_dev,Cylin_dev, tex_dev, Mesh_dev, Box_dev);

    int offset = mark[tid];

	float3 CurColor = img_buffer[offset];

	img_buffer[offset] =  CurColor + ( ray[tid].intensity * Color_Dot(Color[tid], addColor) );       //intensity记录着反射系数

	//Color_Dot(CurColor,)
}

extern "C" void cudaPixelShader(float3* img_buffer, int ray_num, float3* intersect, float3* m_Color, float3* norm, lightpot* Light_dev,float3* illums,int* mark, Sphere* Sp_dev, plane* Plan_dev, Cylinder* Cylin_dev, Texture* tex_dev, Mesh* Mesh_dev, Box* Box_dev, LightRay* ray)
{
	int threadNum = 64;
	int blockNum = ( ray_num + threadNum - 1 ) / threadNum ;

	kernelPixelShader<<<blockNum , threadNum>>>(img_buffer, ray_num, intersect, m_Color, norm,  Light_dev, illums, mark, Sp_dev, Plan_dev,Cylin_dev, tex_dev, Mesh_dev, Box_dev, ray);

	cudaError_t error = cudaSuccess;
	error = cudaGetLastError();

	if (error != cudaSuccess)
	{
		printf("cudaPixelShader() failed to launch error = %d, 意思是: %s\n", error, cudaGetErrorString(error));
	}
}

__global__ void kernelGenerateNextLevelRays(float3 *materialInfo, float3* intersect, float3* norm, LightRay* rayOrig,int rayNumber, LightRay* DestRay, int*	mark )
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= rayNumber )
		return;

	mark[tid] = 0;
	
	//load the intersected point
	float3 inPoint = intersect[tid];
	//load the nomal 
	float3 normal = norm[tid];

	//get the intersected material property
	float3 matInfo = materialInfo[tid];
	//get ray direction
	float3 dir = rayOrig[tid].Dir;

	//if there is reflction
	if(matInfo.x>0)
	{
		mark[tid] = 1;
		float3 reflectDir = d_reflect( dir , normal );
		DestRay[tid].Orig = inPoint + reflectDir * eposi;

		reflectDir = Vec_Normlize( reflectDir );

		DestRay[tid].Dir = reflectDir;
		
		DestRay[tid].intensity = matInfo.x;          //反射率
		DestRay[tid].order = rayOrig[tid].order;

	}

    //if there is a refraction
	else if(matInfo.y>0)
	{
		mark[tid] = 1;
		float3 refractDir = d_refract( dir , normal , matInfo.y );
		refractDir = Vec_Normlize( refractDir );

		DestRay[tid].Dir = refractDir;
		DestRay[tid].Orig = inPoint + refractDir * eposi;
        DestRay[tid].intensity = matInfo.z;         //物体透明度

		DestRay[tid].order = rayOrig[tid].order;
	}

}


extern "C" void cudaGenerateNextLevelRays(	float3 *materialInfo,        //物体反射率，透明度
										  float3*	intersect, 
										  float3*	norm, 
										  LightRay* rayOrig,
										  int		rayNumber ,
										  LightRay* DestRay,
										  int*	mark )
{
	//the block and thread number
	int threadNum = 256;
	int blockNum = ( rayNumber + threadNum - 1 ) / threadNum ;

	//call the kernel
	kernelGenerateNextLevelRays<<<blockNum , threadNum>>>(	materialInfo, intersect, norm, rayOrig, rayNumber,DestRay, mark );

}

__global__ void kernelScan( int* data , int number , int oBlockRes , int* blockRes )
{
	//the shared memory
	__shared__ int sharedMem[512];

	//get the thread id
	int ltid = threadIdx.x;
	int gtid = ltid + blockDim.x * blockIdx.x;

	//the block sum
	int blocksum = 0;

	//zero the rest of the memory
	if( 2 * gtid >= number )
	{
		data[ 2 * gtid ] = 0;
		data[ 2 * gtid + 1 ] = 0;
	}else if( 2 * gtid + 1 == number  )
		data[ 2 * gtid + 1 ] = 0;

	//Load the data into the shared memory
	sharedMem[2*ltid] = data[2*gtid];
	sharedMem[2*ltid+1] = data[2*gtid+1];

	//the offset
	int offset = 1;

	for( int d = 256 ; d > 1 ; d >>= 1 )
	{
		//sync the threads in a group
		__syncthreads();

		if( ltid < d )
		{
			int ai = offset * ( 2 * ltid + 1 ) - 1;
			int bi = ai + offset;

			sharedMem[bi] += sharedMem[ai];
		}

		offset *= 2;
	}

	//the block sum
	blocksum = sharedMem[511] + sharedMem[255];

	//clear the last element
	if( ltid == 0 )
	{
		sharedMem[511] = sharedMem[255];
		sharedMem[255] = 0;
	}

	for( int d = 2 ; d < 512 ; d *= 2 )
	{
		__syncthreads();

		offset >>= 1;

		if( ltid < d )
		{
			int ai = offset * ( 2 * ltid + 1 ) - 1 ;
			int bi = ai + offset ;

			int t = sharedMem[ai];
			sharedMem[ai] = sharedMem[bi];
			sharedMem[bi] += t;
		}
	}

	__syncthreads();

	data[ 2 * gtid ] = sharedMem[ 2 * ltid ];
	data[ 2 * gtid + 1 ] = sharedMem[ 2 * ltid + 1 ];

	//Output Block Result
	if( oBlockRes > 0 )
	{
		if( ltid == 0 )
		{
			//copy the result
			blockRes[blockIdx.x] = blocksum;
		}
	}
}

//Add the block result to the segmented scan result
__global__ void kernelUniformAdd( int* data , int* blockResult )
{
	//get the thread id
	int ltid = threadIdx.x;
	int gtid = ltid + blockDim.x * blockIdx.x;

	//add the result
	data[gtid] += blockResult[gtid/512];
}

//do scan on gpu
extern "C" void cudaScan( int* mark , int num , int level )
{
	//the dimension of the kernel
	dim3 threads( 256 );
	dim3 blocks( ( num + 511 ) / 512 );

	//call the kernel
	kernelScan<<<blocks , threads>>>( mark , num , 1 , g_ScanSum[level] );

	//scan the block Result
	if( num <= 262144 )        //512*512
		kernelScan<<<1 , threads>>>( g_ScanSum[level] , blocks.x , -1 , mark );
	else
		cudaScan( g_ScanSum[level] , blocks.x , level + 1 );

	//add the offset
	threads.x = 512;
	kernelUniformAdd<<< blocks , threads >>> ( mark, g_ScanSum[level] );

}

__global__ void kernelCopyNewRays(LightRay *rayOrig, int* scanResult,int rayNumber,LightRay* Raydest,int* mark)
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= rayNumber )
		return;

	//load the offset
	int offset = scanResult[tid];

	if( offset != scanResult[tid+1] )
	{
		//set the result
		Raydest[offset].Orig = rayOrig[tid].Orig;
		Raydest[offset].Dir  = rayOrig[tid].Dir;
		Raydest[offset].intensity = rayOrig[tid].intensity;
		Raydest[offset].order = rayOrig[tid].order;
		mark[offset] = rayOrig[tid].order;          //保存原来光线的序号
	}
}

//copy new rays
extern "C" void cudaCopyNewRays(	LightRay *rayOrig,
								int* scanResult , 
								int	rayNumber , 
								LightRay* Raydest,
								int* mark )
{
	//the block and thread number
	int threadNum = 256;
	int blockNum = ( rayNumber + threadNum - 1 ) / threadNum ;

	//call the kernel
	kernelCopyNewRays<<<blockNum , threadNum>>>( rayOrig, scanResult, rayNumber , Raydest, mark );
}

//clear the noise of the image
__global__ void kernelClearNoise(	float3* imgData ,  int	width , int	height, float3* targetData )
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= width * height )
		return;

	//threshold
	float threshold = 0.2f;    //设为0的话看起来有种模糊感

	//the difference
	int difference = 0;

	//current index
	int	currentIndex = tid;
	int leftIndex = tid - 1;
	int rightIndex = tid + 1;
	int upIndex = tid - width ;
	int downIndex = tid + width ;

	//the coordinate
	int i = tid % width;
	int j = tid / width;

	//current color
	float3 color = imgData[currentIndex];

	float3 sum = make_float3( 0 , 0 , 0 );
	if( i > 0 )
	{
		if( moduleSqr( color - imgData[leftIndex] ) > threshold )
			difference++;
		sum = sum + imgData[leftIndex];
	}
	if( i < width - 1 )
	{
		if( moduleSqr( color - imgData[rightIndex] ) > threshold )
			difference++;
		sum = sum + imgData[rightIndex];
	}
	if( j > 0 )
	{
		if( moduleSqr( color - imgData[upIndex] ) > threshold )
			difference++;
		sum = sum + imgData[upIndex];
	}
	if( j < height - 1 )
	{
		if( moduleSqr( color - imgData[downIndex] ) > threshold )
			difference++;
		sum = sum + imgData[downIndex];
	}

	if( difference >= 2 )
		color = sum * 0.25f;

	targetData[tid] = color;
}

void cudaClearNoise(	float3* imgData , int width ,	int	height , float3* targetData )
{
	//the block and thread number
	int threads = 256;
	int blocks = ( width * height + 255 ) / 256;

	//call the kernel
	kernelClearNoise<<<blocks , threads>>>( imgData , width , height , targetData );

}


__global__ void kernelCopyImageBuffer( float3* img_buffer, unsigned char* dev_map, int pixelNum )
{
	//get the thread id
	int tid = threadIdx.x + blockIdx.x * blockDim.x;

	//limit the thread id
	if( tid >= pixelNum )
		return;

	img_buffer[tid] = Color_Clamp(img_buffer[tid]);
	dev_map[tid*4] = (int)(img_buffer[tid].x * 255.0);
    dev_map[tid*4+1] = (int)(img_buffer[tid].y * 255.0);
	dev_map[tid*4+2] = (int)(img_buffer[tid].z * 255.0);
	dev_map[tid*4+3] = (int)255;
}

extern "C" void cudaCopyImageBuffer(float3* img_buffer, unsigned char* dev_map, int pixelNum )
{
	//the block number
	int threads = 256;
	int blocks = ( pixelNum + threads - 1 ) / threads;

	//call the kenrel
	kernelCopyImageBuffer<<<blocks,threads>>>( img_buffer , dev_map , pixelNum );
}

