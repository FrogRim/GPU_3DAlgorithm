#include "DrawComponent.h"
#include "gl/freeglut.h"
#include "pmp/io/io.h"
#include "pmp/algorithms/normals.h"

struct Vertex3 {
	GLdouble x, y, z;
};

struct AABB {
	Vertex3 lowerPoint;
	Vertex3 upperPoint;
};

struct Faces {
	Vertex3 vertices[3];    //face를 이루는 점 세개
};

class Node {
public:
	
	Node* leftNode = NULL;
	Node* rightNode = NULL;
	Node* parentNode = NULL;
	AABB Bound;		//리프 노드와 비교하기 위한 현재 노드의 bounding box
	int Level;
	Faces face;

	Node(AABB Bound, int Level)		// root/internal node 생성자
	{
		this->Bound = Bound;
		this->Level = Level;
	}

	Node(Faces Face, AABB Bound, int Level)
	{
		this->face = Face;
		this->Bound = Bound;
		this->Level = Level;
	}


};

Vertex3 Min(Vertex3 A, Vertex3 B)
{
	Vertex3 C;
	C.x = (A.x < B.x) ? A.x : B.x;
	C.y = (A.y < B.y) ? A.y : B.y;
	C.z = (A.z < B.z) ? A.z : B.z;

	return C;
}


Vertex3 Max(Vertex3 A, Vertex3 B)
{
	Vertex3 C;
	C.x = (A.x > B.x) ? A.x : B.x;
	C.y = (A.y > B.y) ? A.y : B.y;
	C.z = (A.z > B.z) ? A.z : B.z;

	return C;
}

int LongestAxis(AABB bound);		
AABB ftoAABB(Faces face);		
void putNode(Node* tree, Node* node);
Node createNode(Faces face);
Node createRoot(pmp::SurfaceMesh mesh);
