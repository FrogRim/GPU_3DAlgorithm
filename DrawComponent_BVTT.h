//���
#pragma once
#include "pmp/surface_mesh.h"
#include "gl/freeglut.h"
#include <stdexcept>
#include <queue>
#include <vector>

// ������ ����
//class BVH;

class DrawComponent
{
public:
    void Init();

    void Draw();

    pmp::SurfaceMesh mesh1;
    pmp::SurfaceMesh mesh2;

    class BVH* bvh1 = nullptr;  // BVH ��ü ������
    class BVH* bvh2 = nullptr;  // BVH ��ü ������

	class BVTT* bvtt = nullptr;  // BVTT ��ü ������

    int arrowNum0 = 0;
    int arrowNum1 = 0;

    ~DrawComponent() {  // �Ҹ���
        delete bvh1;  // BVH ��ü ����
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
    // �⺻ ������
    AABB(pmp::Point pmin, pmp::Point pmax) : pmin(pmin), pmax(pmax) {} // �ּ����� �ִ������� �ʱ�ȭ

    pmp::Point pmin; // �ּ� ��ǥ
    pmp::Point pmax; // �ִ� ��ǥ

    // �ٸ� AABB���� ���� ���� �Ǵ�
    bool intersect(const AABB& other);

    // Ư�� ������ ���� ���� �Ǵ�
    bool intersect(const pmp::Point& p);

    // Ư�� ������ ���� ���͸� ���� ���� ���� �Ǵ�
    bool intersect(const pmp::Point& p, const pmp::Point& d);

    // �ٸ� AABB�� ��ħ
    AABB merge(const AABB& other);

    // ���� �� �� ��ȯ
    int longestAxis();

    // �߽��� ��ȯ
    pmp::Point center();

    // ��带 �� ���� ����
    void split(AABB& left, AABB& right);
};

class Node {
public:
    AABB aabb; // AABB ��ü
    std::vector<int> face_index; // �� �ε��� ���
    Node* left; // ���� �ڽ� ���
    Node* right; // ������ �ڽ� ���
    int level; // ��� ����
    float color[3]; // ����

    Node() : left(nullptr), right(nullptr) {} // �⺻ ������
};

class BVH
{
public:
    int cnt = 0;
    explicit BVH(pmp::SurfaceMesh& mesh) : mesh(mesh), root(nullptr) {} // ������: �޽� ������ BVH �ʱ�ȭ
    int maxDepth = 8; // ��� ���� ���Ѽ��� -> �����÷ο� ����
    pmp::SurfaceMesh& mesh; // �޽� ����
    Node* root; // ��Ʈ ���

    void build();

    void draw(int arrowNum0, int arrowNum1); // BVH�� �ð�ȭ�ϱ� ���� �׸�
    void validate(); // BVH ��ȿ�� �˻�

private:
    Node* buildRecursive(const std::vector<int>& faces, int level);

    void setColor(Node* node, int level);

    AABB computeAABB(int face_idx);

    void drawAABB(const AABB& box, float color[3]);

    void drawRecursive(Node* node, int currentLevel, int arrowNum0, int arrowNum1);

    // ����� ��ȿ���� �����ϴ� ��� �Լ�
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
    std::queue<std::pair<Node*, Node*>> nodeQueue; // ť�� ����
    std::vector<std::pair<Node*, Node*>> collisionPairs; // �浹 ����Ʈ

};