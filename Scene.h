#include<iostream>
#include<math.h>
#include <limits>               //�����������
#include <cuda_runtime_api.h>
#include <vector_functions.h>
using namespace std;

//����
class Texture
{
public:
	Texture(Color* a_Bitmap, int a_Width, int a_Height);
	Texture(const char* a_FileName);
	~Texture()
	{
		delete m_Bitmap;
	}

	//��ȡ��Ƭ����ɫ
	Color GetTexel(float a_U, float a_V);

	//�����ɫ
	Color* GetBitmap()
	{
		return m_Bitmap;
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

private:
	//������ɫ
	Color *m_Bitmap;
	//����
	int m_Width, m_Height;
};

//����
class Material
{
public:
	Material() : m_Refl(0.0f), m_Diff(0.2f), m_Spec(0.8f), 
		m_Refr(0.0f), m_RefrIndex(1.5f), m_Color(0.2f, 0.2f, 0.2f),
		m_Texture( 0 ),	m_UScale( 1.0f ), m_VScale( 1.0f ),m_transparency(0.0)
	{}

	//����͸����
	void setTrans( float a_Trans)
	{
		m_transparency = a_Trans;
	}
	float getTrans()
	{
		return m_transparency;
	}
	//���÷�����
	void SetRefl(float a_Refl)
	{
		m_Refl = a_Refl;
	}
	float GetRefl()
	{
		return m_Refl;
	}
	//������������
	void SetDiff(float a_Diff)
	{
		m_Diff = a_Diff;
	}
	float GetDiff()
	{
		return m_Diff;
	}
	//����������ɫ
	void SetColor(Color a_Color)
	{
		m_Color = a_Color;
	}
	Color GetColor()
	{
		return m_Color;
	}
	//�������徵�淴����
	void SetSpec(float a_Spec)
	{
		m_Spec = a_Spec;
	}
	//��þ��淴����
	float GetSpec()
	{
		return m_Spec;
	}
	//��������������
	void SetRefr(float a_Refr)
	{
		m_Refr = a_Refr;
	}
	float GetRefr()
	{
		return m_Refr;
	}
	//������������ϵ��
	void SetRefrIndex(float a_RefrIndex)
	{
		m_RefrIndex = a_RefrIndex;
	}
	float GetRefrIndex()
	{
		return m_RefrIndex;
	}
	//������������
	void SetTexture( Texture* a_Texture )
	{ 
		m_Texture = a_Texture;
	}
	Texture* GetTexture()
	{
		return m_Texture;
	}
	//������������UV����
	void SetUVScale( float a_UScale, float a_VScale )
	{ 
		m_UScale = a_UScale;
		m_VScale = a_VScale;
	}
	float GetUScale()
	{
		return m_UScale;
	}
	float GetVScale()
	{
		return m_VScale;
	}

private:
	//����͸����
	float  m_transparency;
	//���巴����
	float m_Refl;
	//������������
	float m_Diff;
	//���徵�淴����
	float m_Spec;
	//����������
	float m_Refr;
	//��������ϵ��
	float m_RefrIndex;
	//������ɫ
	Color m_Color;
	//��������
	Texture *m_Texture;
	//�����������ű���
	float m_UScale, m_VScale;
};

class Light
{
public:
	enum
	{
		POINT = 1,
		AREA
	};
	Light( int a_Type, Vector3& a_Pos, Color& a_Color ) : m_Type( a_Type ), m_Pos( a_Pos ), m_Color( a_Color ), m_Grid( 0 ) {};
	Light( int a_Type, Vector3& a_P1, Vector3& a_P2, Vector3& a_P3, Color& a_Color );
	Vector3& GetPos() { return m_Pos; }
	Vector3& GetCellX() { return m_CellX; }
	Vector3& GetCellY() { return m_CellY; }
	Vector3& GetGrid( int a_Idx ) { return m_Grid[a_Idx]; }
	Color& GetColor() { return m_Color; }
	int GetType() { return m_Type; }
private:
	Vector3 m_Pos, m_CellX, m_CellY;
	Color m_Color;
	int m_Type;
	Vector3* m_Grid;
};

//�������
class Primitive
{
public:
	Primitive() : m_Name(0), m_type(0) {}
	~Primitive()
	{
		delete [] m_Name;
		delete m_Material;
	}

	//��������
	void SetName(const char*);
	//�������
	const char* GetName();
	//��
	virtual int Intersect(Ray&, float&) = 0;
	//��÷���
	virtual Vector3 GetNormal(const Vector3&) = 0;
	//��ð�Χ��
	virtual Cube GetBox() = 0;
	//�������������ཻ
	virtual bool IntersectBox(Cube&) = 0;
	//��÷ָ����
	virtual bool GetSplitPos(float&, float&, int) = 0;

	//�������
	int GetType()
	{
		return m_type;
	}
	//��ò���
	Material* GetMaterial()
	{
		return m_Material;
	}
	// 		//�Ƿ��Դ
	// 		bool isLight()
	// 		{
	// 			return m_Light;
	// 		}
	// 		//��Դ
	// 		virtual void Light(bool a_Light)
	// 		{
	// 			m_Light = a_Light;
	// 		}
	//�����ɫ
	virtual Color GetColor(const Vector3& a_Pos)
	{
		return m_Material->GetColor();
	}

	enum
	{
		SPHERETYPE = 1,
		PLANETYPE,
		BOXTYPE,
		TRIANGLETYPE
	};
protected:
	//�������
	Material *m_Material;
	//��������
	char *m_Name;
	// 		//��Դ
	// 		bool m_Light;
	//����
	int m_type;
};

//ƽ��
class PlanePrim : public Primitive
{
public:
	PlanePrim(Vector3& a_Normal, float D) : m_Plane(Plane(a_Normal, D))
	{
		m_Material = new Material();
		m_UAxis = Vector3( m_Plane.N.y, m_Plane.N.z, -m_Plane.N.x );
		m_VAxis = m_UAxis.Cross( m_Plane.N );
		m_type = 2;
	};

	//��
	int Intersect(Ray&, float&);
	//��ð�Χ��
	Cube GetBox();
	//�������������ཻ
	bool IntersectBox(Cube&);
	//�����ɫ
	Color GetColor(const Vector3& a_Pos);
	//�ָ����
	bool GetSplitPos(float&, float&, int);

	//���d
	float GetD()
	{
		return m_Plane.D;
	}
	//��÷���
	Vector3 GetNormal(const Vector3& a_Pos)
	{
		return m_Plane.N;
	}
	//�������
	int GetType()
	{
		return PLANETYPE;
	}

private:
	Plane m_Plane;
	//����UV����
	Vector3 m_UAxis, m_VAxis;
};

//��
class Sphere : public Primitive
{
public:
	Sphere(Vector3 a_Center, float a_Radius) : m_Center(a_Center), m_Radius(a_Radius), m_SqRadius(m_Radius * m_Radius),	m_RRadius(1.0f / m_Radius)
	{
		m_Material = new Material();
		m_type = 1;
	};

	//��
	int Intersect( Ray&, float& );
	//��ð�Χ��
	Cube GetBox();
	//�������������ཻ
	bool IntersectBox(Cube&);
	//�����ɫ
	Color GetColor(const Vector3& a_Pos);
	//�ָ����
	bool GetSplitPos(float&, float&, int);

	//��÷���
	Vector3 GetNormal(const Vector3& a_Pos)
	{
		return (a_Pos - m_Center) * m_RRadius;
	}
	//�������
	Vector3 GetCenter()
	{
		return m_Center;
	}
	//��ð뾶
	float GetRadius()
	{
		return m_Radius;
	}
	//�������
	int GetType()
	{
		return SPHERETYPE;
	}

private:
	//����
	Vector3 m_Center;
	//�뾶���뾶ƽ�����뾶����
	float m_Radius, m_SqRadius, m_RRadius;


};

//������
class Box : public Primitive
{
public:
	Box()
	{
		m_Material = new Material();
		m_type = 3;
	}
	Box(Cube& a_Box) : m_Box(a_Box)
	{
		m_Material = new Material();
		m_type = 3;
	}
	~Box()
	{
	}

	//��
	int Intersect(Ray&, float&);
	//ȡ�÷���
	Vector3 GetNormal(const Vector3&);
	//��ð�Χ��
	Cube GetBox();
	//�������������ཻ
	bool IntersectBox(Cube&);
	// 		//���õƹ��Grid
	// 		void Light(bool a_Light);
	//�����ɫ
	Color GetColor(const Vector3& a_Pos);
	//�ָ����
	bool GetSplitPos(float&, float&, int);

	//�������
	int GetType()
	{
		return BOXTYPE;
	}
	//ȡ������
	Vector3& GetPos()
	{
		return m_Box.GetPos();
	}
	//ȡ�ô�С
	Vector3& GetSize()
	{
		return m_Box.GetSize();
	}
	// 		//���x����ƹ�Grid
	// 		float GetGridX( int a_Idx )
	// 		{
	// 			return m_Grid[a_Idx * 2];
	// 		}
	// 		//���z����ƹ�Grid
	// 		float GetGridZ( int a_Idx )
	// 		{
	// 			return m_Grid[a_Idx * 2 + 1];
	// 		}
	bool Contains(const Vector3& a_Pos)
	{
		return m_Box.Contains(a_Pos);
	}

private:
	Cube m_Box;
	// 		//��ƹ��Grid
	// 		float *m_Grid;
};

class Triangle : public Primitive
{
public:
	Triangle(){m_type = 4;}
	Triangle(Vertex *a_V1, Vertex *a_V2, Vertex *a_V3);

	//��
	int Intersect(Ray&, float&);
	//��÷���
	Vector3 GetNormal(const Vector3&);
	//��ð�Χ��
	Cube GetBox();
	//�������������ཻ
	bool IntersectBox(Cube&);
	//�����ɫ
	Color GetColor(const Vector3& a_Pos);
	//�ָ����
	bool GetSplitPos(float&, float&, int);

	//��ö���
	Vertex* GetVertex(int a_idx)
	{
		return m_Vertex[a_idx];
	}
	//�������
	int GetType()
	{
		return TRIANGLETYPE;
	}

private:
	//����
	Vector3 m_Normal;
	//3����
	Vertex *m_Vertex[3];
	//UV
	float m_U, m_V;
	//����UV����
	float m_Nu, m_Nv, m_Nd;
	//�����������
	int m_K;
	//Ԥ�����AB���м���
	float m_BNu, m_BNv;
	//Ԥ�����AC���м���
	float m_CNu, m_CNv;
	//������xyz����ֵ
	float m_Xmin, m_Xmax, m_Ymin, m_Ymax, m_Zmin, m_Zmax;
};

//��������
class PrimList
{
public:
	PrimList() : m_Primitive(0), m_Next(0) {}
	~PrimList()
	{
		delete m_Next;
	}

	//��������
	void SetPrim(Primitive *a_Prim)
	{
		m_Primitive = a_Prim;
	}
	Primitive* GetPrim()
	{
		return m_Primitive;
	}
	//����next
	void SetNext(PrimList *a_Next)
	{
		m_Next = a_Next;
	}
	PrimList *GetNext()
	{
		return m_Next;
	}

private:
	Primitive *m_Primitive;
	PrimList *m_Next;
};