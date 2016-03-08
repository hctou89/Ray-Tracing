
#include"Interface.h"

float3 Color_RGB( float r, float g, float b)
{
	return make_float3( (float)r/255.0, (float)g/255.0, (float)b/255.0 );
}






void main( )
{
	CPUBitmap bitmap(WIDTH,HEIGHT);

	unsigned char* dev_bitmap;
	cudaMalloc((void**)&dev_bitmap, bitmap.image_size());
    

	Sphere *Sp = (Sphere*)malloc( NUM_SP * sizeof(Sphere));                        //测试球面
    
	lightpot *Light = (lightpot*)malloc(NUM_LIGHT * sizeof(lightpot) );
	plane *MyPlane = (plane*)malloc(NUM_PLANE * sizeof(plane));

	Cylinder *Cylind = (Cylinder*)malloc( NUM_CYLINDER * sizeof(Cylinder) );                  //圆柱

	Texture *MyTexture = (Texture*)malloc( NUM_TEXTURE * sizeof(Texture) );	

	Mesh* MyMesh = (Mesh*)malloc(NUM_MESH * sizeof(Mesh));

	Box* MyBox = (Box*)malloc(NUM_BOX * sizeof(Box));

	float rotate_angle[2][3]= {{0.0 * PI, 0.0 * PI , 0.0 * PI },{0.0 * PI, 0.0 * PI , 0.7 * PI }};

	MyMesh[1].Load3DS("Mesh/cube1.3DS", make_float3(10, 10, 6.0),  3, rotate_angle[0], Color_RGB(255,255,240), 0.5, 0.5, 0);
	MyMesh[0].Load3DS("Mesh/bunny1k.3DS", make_float3(6,8,7.1), 4, rotate_angle[1], Color_RGB(255,215,0), 0.5, 0.5,0);
	//	MyMesh[1].Load3DS("Mesh/knot.3DS", make_float3(5,5,0), 4,  rotate_angle, Color_RGB(255,255,0), 0.5,0.5,  0 );

//	cout<<MyMesh[1].kd<<" "<<MyMesh[1].patch_num<<endl;

	Sphere *dev_Sp;
	plane *dev_plan;
	Texture *dev_texture;
	Mesh *dev_MyMesh;
	Cylinder *dev_Cylind;
	Box *dev_Box;
	lightpot *dev_Light;
	LightRay *dev_ray, *Sec_ray;
	float3* m_cImageBuffer;
	float3 *IntersectPoint;        
    float3* IntersectColor;
	float3 *IntersectNorm, *IntersectIllum, *IntersectMaterial;
	int *m_mark;


	float3 *dev_eye, *dev_lookat;
	float3 *eye = (float3*)malloc( sizeof(float3) );
	float3 *lookat = (float3*)malloc( sizeof(float3) );

 	eye->x = 18, eye->y= 28 ,eye->z= 13;
 	lookat->x = 5, lookat->y = 0, lookat->z=6;

	cudaMalloc((void**)&dev_eye, sizeof(float3) );
	cudaMalloc((void**)&dev_lookat, sizeof(float3) );

	cudaMemcpy(dev_eye, eye, sizeof(float3), cudaMemcpyHostToDevice);
	cudaMemcpy(dev_lookat, lookat, sizeof(float3), cudaMemcpyHostToDevice);

	cudaMalloc((void**)&dev_Light,   NUM_LIGHT * sizeof(lightpot));

	cudaMalloc( (void**)&dev_Sp,        NUM_SP * sizeof(Sphere) ); //球
     
	cudaMalloc((void**)&dev_plan,        NUM_PLANE * sizeof(plane) );       //平面
	cudaMalloc((void**)&dev_texture,        NUM_TEXTURE * sizeof(Texture));
    cudaMalloc((void**)&dev_MyMesh,        NUM_MESH * sizeof(Mesh));
	cudaMalloc( (void**)&dev_Cylind,   NUM_CYLINDER * sizeof(Cylinder) );	
    cudaMalloc( (void**)&dev_Box,      NUM_BOX * sizeof(Box));

	cudaMalloc( (void**)&dev_ray,        WIDTH * HEIGHT * sizeof(LightRay));
	cudaMalloc( (void**)&Sec_ray,        WIDTH * HEIGHT * sizeof(LightRay));

	cudaMalloc( (void**)&IntersectPoint , sizeof( float3 ) * WIDTH * HEIGHT);
	cudaMalloc( (void**)&IntersectColor , sizeof( float3 ) * WIDTH * HEIGHT);

    cudaMalloc( (void**)&IntersectNorm, sizeof( float3 ) * WIDTH * HEIGHT);
	cudaMalloc( (void**)&IntersectIllum, sizeof( float3 ) * WIDTH * HEIGHT);
	cudaMalloc( (void**)&IntersectMaterial, sizeof( float3 ) * WIDTH * HEIGHT);
	cudaMalloc( (void**)&m_mark, sizeof(int) * WIDTH * HEIGHT);

	cudaMalloc( (void**)&m_cImageBuffer, sizeof(float3) * WIDTH * HEIGHT);
	
	cudaMalloc( (void**)&g_ScanSum[0] , sizeof( int ) * 262144 );
	cudaMalloc( (void**)&g_ScanSum[1] , sizeof( int ) * 512 );
	
	Light[0] = lightpot(make_float3(0.7,0.7,0.7),make_float3(10,10,19)); //光源亮度，位置
	Light[1] = lightpot(make_float3(0.6,0.6,0.6),make_float3(20,6,19)); //光源亮度，位置
	Light[2] = lightpot(make_float3(0.6,0.6,0.6),make_float3(20,18,19)); //光源亮度，位置


 	Sp[0] = Sphere(make_float3(13,8,2),/*make_float3(12,8,2),*/ Color_RGB(180,180,180),2, 0.3,0.7, -1, 0.1, 0.1, 0 ,1.1, 0.6);  //透明

 	Sp[1] = Sphere(make_float3(6,14,2), Color_RGB(192,192,192),2, 0.5,0.5,-1, 0.1,0.1,  1.0,0,0);       //镜面

	Sp[2] = Sphere(make_float3(20,10,9.5),/*make_float3(20,10,9.5),*/ Color_RGB(251,251,251), 2, 0.5, 0.5, 5, 1.0, 0.7/1.6, 0, 0, 0 );    //the earth

	MyPlane[0] = plane(make_float3(0,0,1),0,Color_RGB(251,251,251),0.5,0.5,0,0,0,0.15,0.15,0);  //地面  
	MyPlane[1] = plane(make_float3(0,1,0),0,Color_RGB(130,130,130),0.5,0.5,0.7,0,0,1.0,1.0,-1);  //左面  镜子

	MyPlane[2] = plane(make_float3(1,0,0),0,Color_RGB(251,251,251),0.5,0.5,0,0,0,0.1,0.1,4); //前面   
	MyPlane[3] = plane(make_float3(-1,0,0),30.0,Color_RGB(251,255,251),0.5,0.5,0,0,0,0.1,0.1,2);   //back 

	MyPlane[4] = plane(make_float3(0,-1,0),30.0,Color_RGB(200,200,190),0.5,0.5,0,0,0,0.1,0.08,1);   // 右面 
	MyPlane[5] = plane(make_float3(0,0,-1),21.0,Color_RGB(255,215,0),0.5,0.5,0.0,0,0,0.1,0.1,3);   //  上面  


    MyTexture[0].CreatTx( "textures/floor.tga" );
	MyTexture[1].CreatTx("textures/wall6.tga");
	MyTexture[2].CreatTx("textures/wall3.tga" );
	MyTexture[3].CreatTx("textures/wood.tga" );
	MyTexture[4].CreatTx("textures/wall5.tga" );
	MyTexture[5].CreatTx("textures/world.tga" );
	MyTexture[6].CreatTx("textures/dalishi.tga");
    MyTexture[7].CreatTx("textures/katong.tga");
	MyTexture[8].CreatTx("textures/lenna.tga");

	Cylind[0] = Cylinder( 2.0, make_float3(6.5,8,0), 5, 0.5, 0.5, Color_RGB(255,255,240), 1, 1, 7, 0 );  //兔子座
	Cylind[1] = Cylinder( 4.0,  make_float3(0,0,0),  30,  0.6, 0.5, Color_RGB(255,255,240), 1, 1, 6, 0 ); //柱子

	Cylind[2] = Cylinder( 0.5,  make_float3(20,10,0 ),  7,  0.5, 0.5, Color_RGB(255,255,240), 1, 1, 6, 0 ); //桌子脚
	Cylind[3] = Cylinder( 4.0,  make_float3(20,10,7.0),   0.5,  0.5, 0.5, Color_RGB(255,255,240), 1, 1, 6, 0 ); //桌面

//	MyBox[0] = Box(make_float3(0,8,8),  make_float3(0.1, 8, 6), Color_RGB(130,130,130), 0.5, 0.5, 0.7,0, 1,1,-1, 0);
	MyBox[0] = Box(make_float3(0,8,8),  make_float3(0.1, 8, 6), Color_RGB(192,192,192), 0.8, 0.5, 0.0, 0, 1, 1,8, 0);

	cudaMemcpy(dev_Light,    Light,    NUM_LIGHT * sizeof(lightpot),cudaMemcpyHostToDevice);
    cudaMemcpy(dev_Sp,       Sp,      NUM_SP * sizeof(Sphere),   cudaMemcpyHostToDevice );
	cudaMemcpy(dev_plan,    MyPlane,   NUM_PLANE * sizeof(plane),  cudaMemcpyHostToDevice);
	cudaMemcpy(dev_texture, MyTexture, NUM_TEXTURE * sizeof(Texture), cudaMemcpyHostToDevice);
    cudaMemcpy(dev_MyMesh,  MyMesh,    NUM_MESH * sizeof(Mesh),   cudaMemcpyHostToDevice);
    cudaMemcpy(dev_Cylind,   Cylind,  NUM_CYLINDER * sizeof(Cylinder), cudaMemcpyHostToDevice );
   cudaMemcpy(dev_Box,       MyBox,   NUM_BOX * sizeof(Box),   cudaMemcpyHostToDevice);

	int raynum = WIDTH * HEIGHT;

	int travel_level = 0;
	
	cudaEvent_t  start,stop;
	cudaEventCreate(&start);
	cudaEventCreate(&stop);
	cudaEventRecord(start,0);

	cudaGeneratePrimaryRays(dev_eye, dev_lookat, WIDTH , HEIGHT, dev_ray);

	//initialize buffer
	cudaInitBuffer( m_cImageBuffer, m_mark , WIDTH * HEIGHT );

	while(travel_level < Travel_Depth && raynum > 0)
	{
		cudaGetIntersect(dev_ray, raynum, IntersectPoint, IntersectColor, IntersectNorm, IntersectIllum, IntersectMaterial,dev_texture, dev_Sp, dev_plan, dev_Cylind, dev_MyMesh, dev_Box);

		cudaPixelShader(m_cImageBuffer, raynum, IntersectPoint, IntersectColor, IntersectNorm, dev_Light, IntersectIllum, m_mark,dev_Sp, dev_plan, dev_Cylind, dev_texture, dev_MyMesh,dev_Box, dev_ray );

		cudaGenerateNextLevelRays(IntersectMaterial, IntersectPoint, IntersectNorm, dev_ray, raynum, Sec_ray,m_mark );
	
		cudaScan( m_mark , raynum + 1 );

		//get the number of rays
		int newrayNum;
		cudaMemcpy( &newrayNum , m_mark + raynum , sizeof(int) , cudaMemcpyDeviceToHost );
	 	if( newrayNum == 0 )
	 		break;

		//copy the new rays
		cudaCopyNewRays( Sec_ray, m_mark, raynum, dev_ray, m_mark );

		//update the ray number
		raynum = newrayNum;

		//update current trace level
		travel_level++;
	}

	cudaClearNoise(m_cImageBuffer, WIDTH, HEIGHT, m_cImageBuffer );

	cudaCopyImageBuffer(m_cImageBuffer, dev_bitmap, WIDTH * HEIGHT);

	cudaMemcpy(bitmap.get_ptr(),dev_bitmap, bitmap.image_size(),cudaMemcpyDeviceToHost);

	cudaEventRecord(stop,0);
	cudaEventSynchronize(stop);
	float elapsedTime;
	cudaEventElapsedTime(&elapsedTime,start,stop);
	cout<<"Time to generate: "<<elapsedTime<<" ms"<<endl;

	bitmap.display_and_exit();

	free(Light);
    free(Sp);
    free(Cylind);
	free(MyPlane);
	free(MyTexture);
    free(MyMesh);
    free(MyBox);
	free(eye);
	free(lookat);

	cudaFree(dev_eye);
	cudaFree(dev_lookat);
    
	cudaFree(dev_bitmap);

	cudaFree(dev_Light);

	cudaFree(dev_Sp);
	cudaFree(dev_plan);
	cudaFree(dev_texture);
	cudaFree(dev_Cylind);
    cudaFree(dev_MyMesh);
	cudaFree(dev_ray);
	cudaFree(Sec_ray);
	cudaFree(IntersectPoint);
	cudaFree(IntersectColor);
	cudaFree(IntersectNorm);
	cudaFree(IntersectIllum);
	cudaFree(m_mark);
	cudaFree(dev_plan);
    cudaFree(dev_Box);

	cudaFree(g_ScanSum[0]);
	cudaFree(g_ScanSum[1]);
	cudaFree(m_cImageBuffer);
}