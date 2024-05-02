#include "bvh.h"
#include <iostream>

//가장 긴 축 구하기
int LongestAxis(AABB bound)
{
	GLdouble x = bound.upperPoint.x - bound.lowerPoint.x;
	GLdouble y = bound.upperPoint.y - bound.lowerPoint.y;
	GLdouble z = bound.upperPoint.z - bound.lowerPoint.z;

	if (x > y && x > z) return 0;
	if (y > x && y > z) return 1;
	return 2;
}

//face로 AABB boundary 구하기
AABB ftoAABB(Faces face)
{
	AABB a;
	a.lowerPoint = Min(Min(face.vertices[0], face.vertices[1]), face.vertices[2]);
	a.upperPoint = Max(Max(face.vertices[0], face.vertices[1]), face.vertices[2]);

	return a;
}


//AABB 자르기
AABB cutAABB(AABB bound)
{
	AABB temp = bound;
	int longest = LongestAxis(bound);
	if (longest == 0)
		temp.lowerPoint.x = (bound.upperPoint.x + bound.lowerPoint.x) / 2;
	if(longest == 1)
		temp.lowerPoint.y = (bound.upperPoint.y + bound.lowerPoint.y) / 2;
	if(longest == 2)
		temp.lowerPoint.z = (bound.upperPoint.z + bound.lowerPoint.z) / 2;

	return temp;
}


//face가 영역 안에 포함되는지 확인(vertex 두 개 이상)
bool checkBound(AABB bound, Faces face)
{
	int count = 0;
	for (int i = 0; i < 3; i++)
	{
		if (face.vertices[i].x >= bound.lowerPoint.x && face.vertices[i].x <= bound.upperPoint.x
			&& face.vertices[i].y >= bound.lowerPoint.y && face.vertices[i].y <= bound.upperPoint.y
			&& face.vertices[i].z >= bound.lowerPoint.z && face.vertices[i].z <= bound.upperPoint.z)
			count++;
	}
	if (count > 2)
		return true;
	else
		return false;
}


//트리에 node 삽입
void putNode(Node* tree, Node* node)
{
	AABB tempAABB = cutAABB((*tree).Bound);
	if (checkBound(tempAABB, (*node).face))		//윗 bound에 포함되는 지
	{
		Node temp(tempAABB, (*tree).Level + 1);
		(*tree).leftNode = &temp;
		//(*tree).leftNode->parentNode = tree;
		putNode((*tree).leftNode, node);
	}
	else				//bound 영역 아래쪽으로 수정
	{
		tempAABB.upperPoint.x = tempAABB.lowerPoint.x;
		tempAABB.upperPoint.y = tempAABB.lowerPoint.y;
		tempAABB.upperPoint.z = tempAABB.lowerPoint.z;
		tempAABB.lowerPoint.x = (*tree).Bound.lowerPoint.x;
		tempAABB.lowerPoint.y = (*tree).Bound.lowerPoint.y;
		tempAABB.lowerPoint.z = (*tree).Bound.lowerPoint.z;
		if (checkBound(tempAABB, (*node).face))	//아랫 bound에 포함되는 지
		{
			(*tree).rightNode = new Node(tempAABB, (*tree).Level + 1);
			//(*tree).rightNode->parentNode = tree;
			putNode((*tree).rightNode, node);
		}	
	}
	(*tree).face = (*node).face;
	(*tree).Bound = (*node).Bound;
}

//leaf node 생성
Node createNode(Faces face)
{
	Node temp(face, ftoAABB(face), 0);
	return temp;
}

//root node 생성
Node createRoot(pmp::SurfaceMesh mesh)   //최대 bound로 
{
	Node rootNode(biggestAABB(mesh), 0);
	return rootNode;
}

//최초의 가장 큰 AABB 구하기
AABB biggestAABB(pmp::SurfaceMesh mesh)
{
	AABB a;
	GLdouble Max_upper_x = nan("1"), Max_upper_y, Max_upper_z;   //처음인지 확인하기 위한 nan
	GLdouble Min_lower_x, Min_lower_y, Min_lower_z;

	for (auto f : mesh.faces()) {		
		for (auto v : mesh.vertices(f)) {
			auto p = mesh.position(v);  

			if (!isnan(Max_upper_x))			//값이 들어있으면 좌표 차례대로 넣으면서 최대 최소 좌표 찾기
			{
				Max_upper_x = (Max_upper_x > p[0]) ? Max_upper_x : p[0];
				Max_upper_y = (Max_upper_y > p[0]) ? Max_upper_y : p[1];
				Max_upper_z = (Max_upper_z > p[0]) ? Max_upper_z : p[2];
				Min_lower_x = (Min_lower_x < p[0]) ? Min_lower_x : p[0];
				Min_lower_y = (Min_lower_y < p[0]) ? Min_lower_y : p[0];
				Min_lower_z = (Min_lower_z < p[0]) ? Min_lower_z : p[0];
			}
			else							//값이 없으면 최초로 받은 좌표값으로 통일
			{
				Max_upper_x = p[0];
				Max_upper_y = p[1];
				Max_upper_z = p[2];
				Min_lower_x = p[0];
				Min_lower_y = p[1];
				Min_lower_z = p[2];
			}

		}
	}

	a.upperPoint.x = Max_upper_x;
	a.upperPoint.y = Max_upper_y;
	a.upperPoint.z = Max_upper_z;
	a.lowerPoint.x = Min_lower_x;
	a.lowerPoint.y = Min_lower_y;
	a.lowerPoint.z = Min_lower_z;
	
	return a;
}


