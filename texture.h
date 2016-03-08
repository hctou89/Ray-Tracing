#include"common.h"




class Texture
{
public:
	Texture(){ cpu_Bitmap = NULL; cuda_Bitmap = NULL; 	m_Width = 0;	m_Height = 0;}
	Texture(const char* a_FileName);
	Texture(float3* a_Bitmap, int a_Width, int a_Height);
	void CreatTx(const char* a_FileName);
// 	Texture(const Texture &t){
// 		cout << "Texture copy" << endl;
// 		m_Bitmap = t.m_Bitmap;
// 		m_Width = t.m_Width;
// 		m_Height = t.m_Height;
// 	}

	~Texture()
	{
		cout << "~Texture" << endl;
		if(cpu_Bitmap)
			delete cpu_Bitmap;
		cudaFree( cuda_Bitmap );
	}

	//��ȡ��Ƭ����ɫ
	float3 GetTexel(float a_U, float a_V);

	//�����ɫ
	float3* GetBitmap()
	{
		return cuda_Bitmap;
	}
	//��ȡ����
	int GetWidth()
	{
		return m_Width;
	}
	int GetHeight()
	{
		return m_Height;
	}


	//������ɫ
	float3* cpu_Bitmap;  //CPU�ڴ�

	float3* cuda_Bitmap; //cuda�ڴ�
	//����
	int m_Width, m_Height;


};