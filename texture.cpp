#include"texture.h"



Texture::Texture(float3* a_Bitmap, int a_Width, int a_Height)
{
	cuda_Bitmap = a_Bitmap;
	m_Width = a_Width;
	m_Height = a_Height;
}



 Texture::Texture(const char* a_FileName)
{
	ifstream input;
	input.open(a_FileName, ifstream::binary);
	if(input.fail())
	{
		cout<<"can't open the file!"<<endl;
		return;		
	}
	//从文件头中获取长宽
	char buffer[20];
	input.read(buffer, 20);
	m_Width = *(buffer + 12) + 256 * *(buffer + 13);
	m_Height = *(buffer + 14) + 256 * *(buffer + 15);
	input.close();
	//读像素
	input.open(a_FileName, ifstream::binary);
	char* t = new char[m_Width * m_Height * 3 + 1024];
	input.read(t, m_Width * m_Height * 3 + 1024);
// 	unsigned char *t;
// 	= (unsigned)a_t;
	input.close();
	//转化为RGB颜色
	cpu_Bitmap = new float3[m_Width * m_Height];

	float rec = 1.0f / 256;
	for ( int size = m_Width * m_Height, i = 0; i < size; i++ )
		cpu_Bitmap[i] = make_float3( (unsigned char)t[i * 3 + 20] * rec , (unsigned char)t[i * 3 + 19] * rec , (unsigned char)t[i * 3 + 18] * rec  );
	delete t;

	cout << "Texture" << endl;
}


void  Texture::CreatTx(const char* a_FileName)
 {
	 if(cpu_Bitmap)
		 delete cpu_Bitmap;
	 cpu_Bitmap = NULL;
	 ifstream input;
	 input.open(a_FileName, ifstream::binary);
	 if(input.fail())
	 {
		 cout<<"can't open the file!"<<endl;
		 return;		
	 }
	 //从文件头中获取长宽
	 char buffer[20];
	 input.read(buffer, 20);
	 m_Width = *(buffer + 12) + 256 * *(buffer + 13);
	 m_Height = *(buffer + 14) + 256 * *(buffer + 15);
	 input.close();
	 //读像素
	 input.open(a_FileName, ifstream::binary);
	 char* t = new char[m_Width * m_Height * 3 + 1024];
	 input.read(t, m_Width * m_Height * 3 + 1024);

	 input.close();
	 //转化为RGB颜色
	 cpu_Bitmap = new float3[m_Width * m_Height];

	 //分配GPU内存
	 cudaMalloc( (void**)&cuda_Bitmap, m_Width * m_Height * sizeof(float3));
	 float rec = 1.0f / 256;
	 for ( int size = m_Width * m_Height, i = 0; i < size; i++ )
		 cpu_Bitmap[i] = make_float3( (unsigned char)t[i * 3 + 20] * rec , (unsigned char)t[i * 3 + 19] * rec , (unsigned char)t[i * 3 + 18] * rec  );
	 delete t;

	 cudaMemcpy(cuda_Bitmap, cpu_Bitmap, m_Width * m_Height * sizeof(float3),cudaMemcpyHostToDevice );

	 
//	 cout << "Texture creat" << m_Width<<" "<<m_Height<<endl;

 }