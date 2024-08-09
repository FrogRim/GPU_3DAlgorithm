//헤더
#pragma once
#include "pmp/surface_mesh.h"
#include "gl/freeglut.h"
#include <stdexcept>
#include <queue>
#include <vector>

// 포워드 선언
//class BVH;

class DrawComponent
{
public:
    void Init();

    void Draw();

    pmp::SurfaceMesh mesh1;
    pmp::SurfaceMesh mesh2;

    class BVH* bvh1 = nullptr;  // BVH 객체 포인터
    class BVH* bvh2 = nullptr;  // BVH 객체 포인터

	class BVTT* bvtt = nullptr;  // BVTT 객체 포인터

    int arrowNum0 = 0;
    int arrowNum1 = 0;

    ~DrawComponent() {  // 소멸자
        delete bvh1;  // BVH 객체 삭제
        delete bvh2;
		delete bvtt;
    }
};

class AABB
{
public:
    AABB() {
        pmin = pmp::Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        pmax = pmp::Point(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    }
    // 기본 생성자
    AABB(pmp::Point pmin, pmp::Point pmax) : pmin(pmin), pmax(pmax) {} // 최소점과 최대점으로 초기화

    pmp::Point pmin; // 최소 좌표
    pmp::Point pmax; // 최대 좌표

    // 다른 AABB와의 교차 여부 판단
    bool intersect(const AABB& other);

    // 특정 점과의 교차 여부 판단
    bool intersect(const pmp::Point& p);

    // 특정 점에서 방향 벡터를 따라 교차 여부 판단
    bool intersect(const pmp::Point& p, const pmp::Point& d);

    // 다른 AABB와 합침
    AABB merge(const AABB& other);

    // 가장 긴 축 반환
    int longestAxis();

    // 중심점 반환
    pmp::Point center();

    // 노드를 두 개로 분할
    void split(AABB& left, AABB& right);
};

class Node {
public:
    AABB aabb; // AABB 객체
    std::vector<int> face_index; // 면 인덱스 목록
    Node* left; // 왼쪽 자식 노드
    Node* right; // 오른쪽 자식 노드
    int level; // 노드 레벨
    float color[3]; // 색상

    Node() : left(nullptr), right(nullptr) {} // 기본 생성자
};

class BVH
{
public:
    int cnt = 0;
    explicit BVH(pmp::SurfaceMesh& mesh) : mesh(mesh), root(nullptr) {} // 생성자: 메시 참조로 BVH 초기화
    int maxDepth = 8; // 재귀 깊이 제한설정 -> 오버플로우 방지
    pmp::SurfaceMesh& mesh; // 메시 참조
    Node* root; // 루트 노드

    void build();

    void draw(int arrowNum0, int arrowNum1); // BVH를 시각화하기 위해 그림
    void validate(); // BVH 유효성 검사

private:
    Node* buildRecursive(const std::vector<int>& faces, int level);

    void setColor(Node* node, int level);

    AABB computeAABB(int face_idx);

    void drawAABB(const AABB& box, float color[3]);

    void drawRecursive(Node* node, int currentLevel, int arrowNum0, int arrowNum1);

    // 노드의 유효성을 검증하는 재귀 함수
    bool validateNode(Node* node, Node* parent);
};

class BVTT {

public:
    void build(BVH* bvh1, BVH* bvh2);
    void drawCollisions();

private:
    void processNodePair(Node* node1, Node* node2);
    void handleCollision(Node* node1, Node* node2);
    void drawAABB(const AABB& box)const;

    BVH* bvh1 = nullptr;
    BVH* bvh2 = nullptr;
    std::queue<std::pair<Node*, Node*>> nodeQueue; // 큐로 관리
    std::vector<std::pair<Node*, Node*>> collisionPairs; // 충돌 리스트

};