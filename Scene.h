#include<iostream>
#include<math.h>
#include <limits>               //定义无穷大用
#include <cuda_runtime_api.h>
#include <vector_functions.h>
using namespace std;

//纹理
class Texture
{
public:
	Texture(Color* a_Bitmap, int a_Width, int a_Height);
	Texture(const char* a_FileName);
	~Texture()
	{
		delete m_Bitmap;
	}

	//获取面片点颜色
	Color GetTexel(float a_U, float a_V);

	//获得颜色
	Color* GetBitmap()
	{
		return m_Bitmap;
	}
	//获取长宽
	int GetWidth()
	{
		return m_Width;
	}
	int GetHeight()
	{
		return m_Height;
	}

private:
	//纹理颜色
	Color *m_Bitmap;
	//长宽
	int m_Width, m_Height;
};

//材质
class Material
{
public:
	Material() : m_Refl(0.0f), m_Diff(0.2f), m_Spec(0.8f), 
		m_Refr(0.0f), m_RefrIndex(1.5f), m_Color(0.2f, 0.2f, 0.2f),
		m_Texture( 0 ),	m_UScale( 1.0f ), m_VScale( 1.0f ),m_transparency(0.0)
	{}

	//设置透明度
	void setTrans( float a_Trans)
	{
		m_transparency = a_Trans;
	}
	float getTrans()
	{
		return m_transparency;
	}
	//设置反射率
	void SetRefl(float a_Refl)
	{
		m_Refl = a_Refl;
	}
	float GetRefl()
	{
		return m_Refl;
	}
	//设置漫反射率
	void SetDiff(float a_Diff)
	{
		m_Diff = a_Diff;
	}
	float GetDiff()
	{
		return m_Diff;
	}
	//设置物体颜色
	void SetColor(Color a_Color)
	{
		m_Color = a_Color;
	}
	Color GetColor()
	{
		return m_Color;
	}
	//设置物体镜面反射率
	void SetSpec(float a_Spec)
	{
		m_Spec = a_Spec;
	}
	//获得镜面反射率
	float GetSpec()
	{
		return m_Spec;
	}
	//设置物体折射率
	void SetRefr(float a_Refr)
	{
		m_Refr = a_Refr;
	}
	float GetRefr()
	{
		return m_Refr;
	}
	//设置物体折射系数
	void SetRefrIndex(float a_RefrIndex)
	{
		m_RefrIndex = a_RefrIndex;
	}
	float GetRefrIndex()
	{
		return m_RefrIndex;
	}
	//设置物体纹理
	void SetTexture( Texture* a_Texture )
	{ 
		m_Texture = a_Texture;
	}
	Texture* GetTexture()
	{
		return m_Texture;
	}
	//设置物体纹理UV坐标
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
	//物体透明度
	float  m_transparency;
	//物体反射率
	float m_Refl;
	//物体漫反射率
	float m_Diff;
	//物体镜面反射率
	float m_Spec;
	//物体折射率
	float m_Refr;
	//物体折射系数
	float m_RefrIndex;
	//物体颜色
	Color m_Color;
	//物体纹理
	Texture *m_Texture;
	//物体纹理缩放比例
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

//物体基类
class Primitive
{
public:
	Primitive() : m_Name(0), m_type(0) {}
	~Primitive()
	{
		delete [] m_Name;
		delete m_Material;
	}

	//设置名字
	void SetName(const char*);
	//获得名字
	const char* GetName();
	//求交
	virtual int Intersect(Ray&, float&) = 0;
	//获得法向
	virtual Vector3 GetNormal(const Vector3&) = 0;
	//获得包围盒
	virtual Cube GetBox() = 0;
	//测试与立方体相交
	virtual bool IntersectBox(Cube&) = 0;
	//获得分割面点
	virtual bool GetSplitPos(float&, float&, int) = 0;

	//获得类型
	int GetType()
	{
		return m_type;
	}
	//获得材质
	Material* GetMaterial()
	{
		return m_Material;
	}
	// 		//是否光源
	// 		bool isLight()
	// 		{
	// 			return m_Light;
	// 		}
	// 		//光源
	// 		virtual void Light(bool a_Light)
	// 		{
	// 			m_Light = a_Light;
	// 		}
	//获得颜色
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
	//物体材质
	Material *m_Material;
	//物体名称
	char *m_Name;
	// 		//光源
	// 		bool m_Light;
	//类型
	int m_type;
};

//平面
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

	//求交
	int Intersect(Ray&, float&);
	//获得包围盒
	Cube GetBox();
	//测试与立方体相交
	bool IntersectBox(Cube&);
	//获得颜色
	Color GetColor(const Vector3& a_Pos);
	//分割面点
	bool GetSplitPos(float&, float&, int);

	//获得d
	float GetD()
	{
		return m_Plane.D;
	}
	//获得法向
	Vector3 GetNormal(const Vector3& a_Pos)
	{
		return m_Plane.N;
	}
	//获得类型
	int GetType()
	{
		return PLANETYPE;
	}

private:
	Plane m_Plane;
	//纹理UV坐标
	Vector3 m_UAxis, m_VAxis;
};

//球
class Sphere : public Primitive
{
public:
	Sphere(Vector3 a_Center, float a_Radius) : m_Center(a_Center), m_Radius(a_Radius), m_SqRadius(m_Radius * m_Radius),	m_RRadius(1.0f / m_Radius)
	{
		m_Material = new Material();
		m_type = 1;
	};

	//求交
	int Intersect( Ray&, float& );
	//获得包围盒
	Cube GetBox();
	//测试与立方体相交
	bool IntersectBox(Cube&);
	//获得颜色
	Color GetColor(const Vector3& a_Pos);
	//分割面点
	bool GetSplitPos(float&, float&, int);

	//获得法向
	Vector3 GetNormal(const Vector3& a_Pos)
	{
		return (a_Pos - m_Center) * m_RRadius;
	}
	//获得球心
	Vector3 GetCenter()
	{
		return m_Center;
	}
	//获得半径
	float GetRadius()
	{
		return m_Radius;
	}
	//获得类型
	int GetType()
	{
		return SPHERETYPE;
	}

private:
	//球心
	Vector3 m_Center;
	//半径、半径平方、半径倒数
	float m_Radius, m_SqRadius, m_RRadius;


};

//立方体
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

	//求交
	int Intersect(Ray&, float&);
	//取得法向
	Vector3 GetNormal(const Vector3&);
	//获得包围盒
	Cube GetBox();
	//测试与立方体相交
	bool IntersectBox(Cube&);
	// 		//设置灯光的Grid
	// 		void Light(bool a_Light);
	//获得颜色
	Color GetColor(const Vector3& a_Pos);
	//分割面点
	bool GetSplitPos(float&, float&, int);

	//获得类型
	int GetType()
	{
		return BOXTYPE;
	}
	//取得坐标
	Vector3& GetPos()
	{
		return m_Box.GetPos();
	}
	//取得大小
	Vector3& GetSize()
	{
		return m_Box.GetSize();
	}
	// 		//获得x方向灯光Grid
	// 		float GetGridX( int a_Idx )
	// 		{
	// 			return m_Grid[a_Idx * 2];
	// 		}
	// 		//获得z方向灯光Grid
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
	// 		//面灯光的Grid
	// 		float *m_Grid;
};

class Triangle : public Primitive
{
public:
	Triangle(){m_type = 4;}
	Triangle(Vertex *a_V1, Vertex *a_V2, Vertex *a_V3);

	//求交
	int Intersect(Ray&, float&);
	//获得法向
	Vector3 GetNormal(const Vector3&);
	//获得包围盒
	Cube GetBox();
	//测试与立方体相交
	bool IntersectBox(Cube&);
	//获得颜色
	Color GetColor(const Vector3& a_Pos);
	//分割面点
	bool GetSplitPos(float&, float&, int);

	//获得顶点
	Vertex* GetVertex(int a_idx)
	{
		return m_Vertex[a_idx];
	}
	//获得类型
	int GetType()
	{
		return TRIANGLETYPE;
	}

private:
	//法向
	Vector3 m_Normal;
	//3个点
	Vertex *m_Vertex[3];
	//UV
	float m_U, m_V;
	//法向UV分量
	float m_Nu, m_Nv, m_Nd;
	//法向主轴分量
	int m_K;
	//预计算边AB的中间量
	float m_BNu, m_BNv;
	//预计算边AC的中间量
	float m_CNu, m_CNv;
	//三个点xyz的最值
	float m_Xmin, m_Xmax, m_Ymin, m_Ymax, m_Zmin, m_Zmax;
};

//物体链表
class PrimList
{
public:
	PrimList() : m_Primitive(0), m_Next(0) {}
	~PrimList()
	{
		delete m_Next;
	}

	//设置物体
	void SetPrim(Primitive *a_Prim)
	{
		m_Primitive = a_Prim;
	}
	Primitive* GetPrim()
	{
		return m_Primitive;
	}
	//设置next
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