//헤더
#pragma once
#include "pmp/surface_mesh.h"
#include "gl/freeglut.h"
#include <cassert> 
#include <stack>

// 포워드 선언
class BVH;

class DrawComponent
{
public:
    void Init();

    void Draw();

    pmp::SurfaceMesh mesh;

    BVH* bvh;  // BVH 객체 포인터

    int arrowNum0 = 0;
    int arrowNum1 = 0;

    ~DrawComponent() {  // 소멸자
        delete bvh;  // BVH 객체 삭제
    }
};

class AABB
{
public:
    AABB() {} // 기본 생성자
    AABB(pmp::Point pmin, pmp::Point pmax) : pmin(pmin), pmax(pmax) {} // 최소점과 최대점으로 초기화

    pmp::Point pmin; // 최소 좌표
    pmp::Point pmax; // 최대 좌표

    // 다른 AABB와의 교차 여부 판단
    bool intersect(const AABB& other)
    {
        if (pmax[0] < other.pmin[0] || pmin[0] > other.pmax[0])
            return false;
        if (pmax[1] < other.pmin[1] || pmin[1] > other.pmax[1])
            return false;
        if (pmax[2] < other.pmin[2] || pmin[2] > other.pmax[2])
            return false;
        return true;
    }

    // 특정 점과의 교차 여부 판단
    bool intersect(const pmp::Point& p)
    {
        if (p[0] < pmin[0] || p[0] > pmax[0]) return false;
        if (p[1] < pmin[1] || p[1] > pmax[1]) return false;
        if (p[2] < pmin[2] || p[2] > pmax[2]) return false;
        return true;
    }

    // 특정 점에서 방향 벡터를 따라 교차 여부 판단
    bool intersect(const pmp::Point& p, const pmp::Point& d)
    {
        double tmin = 0.0;
        double tmax = 1.0;
        for (int i = 0; i < 3; i++)
        {
            if (d[i] == 0.0)
            {
                if (p[i] < pmin[i] || p[i] > pmax[i])
                    return false;
            }
            else
            {
                double t1 = (pmin[i] - p[i]) / d[i];
                double t2 = (pmax[i] - p[i]) / d[i];
                if (t1 > t2)
                    std::swap(t1, t2);

                tmin = std::max(tmin, t1);
                tmax = std::min(tmax, t2);

                if (tmin > tmax)
                    return false;
            }
        }
        return true;
    }

    // 다른 AABB와 합침
    AABB merge(const AABB& other)
    {
        pmp::Point newPmin, newPmax;
        for (int i = 0; i < 3; i++)
        {
            newPmin[i] = std::min(this->pmin[i], other.pmin[i]);
            newPmax[i] = std::max(this->pmax[i], other.pmax[i]);
        }
        return AABB(newPmin, newPmax);
    }

    // 가장 긴 축 반환
    int longestAxis()
    {
        pmp::Point d = pmax - pmin;
        if (d[0] > d[1] && d[0] > d[2]) return 0;
        if (d[1] > d[2]) return 1;
        return 2;
    }

    // 중심점 반환
    pmp::Point center()
    {
        return 0.5 * (pmin + pmax);
    }

    // 노드를 두 개로 분할
    void split(AABB& left, AABB& right)
    {
        int axis = longestAxis();
        double mid = (pmax[axis] + pmin[axis]) * 0.5;
        pmp::Point pmin_l = pmin;
        pmp::Point pmax_l = pmax;
        pmp::Point pmin_r = pmin;
        pmp::Point pmax_r = pmax;

        pmax_l[axis] = mid; // 왼쪽 상자의 최대 AABB 수정
        pmin_r[axis] = mid; // 오른쪽 상자의 최소 AABB 수정

        left = AABB(pmin_l, pmax_l);
        right = AABB(pmin_r, pmax_r);
    }
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

    void build()
    {
        std::vector<int> faces;
        for (auto f : mesh.faces()) {
            faces.push_back(f.idx());
        }
        root = buildRecursive(faces, 0, maxDepth);
        std::cout << "BVH build complete." << std::endl;
    }

    void draw(int arrowNum0, int arrowNum1) // BVH를 시각화하기 위해 그림
    {
        drawRecursive(root, 0, arrowNum0, arrowNum1); // 루트에서 시작하여 재귀적으로 그림
    }
    void validate()
    {
        assert(validateNode(root, nullptr)); // 루트 노드부터 검증 시작
    }
private:
    Node* buildRecursive(const std::vector<int>& faces, int level, int maxDepth) {

        if (level > maxDepth || faces.empty()) {
            Node* node = new Node;
            node->level = level;
            node->face_index = faces; // 비어있지 않다면, 이전 면들을 포함

            node->color[0] = static_cast<float>((level * 40) % 255) / 255.0f;
            node->color[1] = static_cast<float>((level * 70) % 255) / 255.0f;
            node->color[2] = static_cast<float>((level * 90) % 255) / 255.0f;

            return node;
        }

        Node* node = new Node;
        node->level = level;

        // 색상 설정 
        node->color[0] = static_cast<float>((level * 40) % 255) / 255.0f;
        node->color[1] = static_cast<float>((level * 70) % 255) / 255.0f;
        node->color[2] = static_cast<float>((level * 90) % 255) / 255.0f;

        // 모든 면의 AABB 계산
        AABB aabb;
        for (auto f : faces) {
            aabb = aabb.merge(computeAABB(f));
        }
        node->aabb = aabb;

        if (faces.size() <= 1) {  // 리프 노드 조건
            node->face_index = faces;
            std::cout << "Leaf node created at level " << level << " with 1 face." << std::endl;
            return node;
        }

        // 면을 좌우 자식 노드로 분할
        AABB leftAABB, rightAABB;
        std::vector<int> leftFaces, rightFaces;
        aabb.split(leftAABB, rightAABB);

        for (auto f : faces) {
            AABB faceAABB = computeAABB(f);
            bool leftIntersect = faceAABB.intersect(leftAABB);
            bool rightIntersect = faceAABB.intersect(rightAABB);

            int axis = aabb.longestAxis();
            double splitPoint = aabb.center()[axis];

            int leftCount = 0, rightCount = 0;
            auto face = pmp::Face(f);
            auto hf = mesh.halfedge(face);
            auto h = hf;
            pmp::Point triangleCenter(0.0, 0.0, 0.0);
            int vertexCount = 0;

            // 삼각형의 각 꼭짓점 위치를 조사하여 분할 규칙을 결정
            do {
                pmp::Vertex v = mesh.to_vertex(h);
                pmp::Point pos = mesh.position(v);
                triangleCenter += pos;
                vertexCount++;
                if (pos[axis] < splitPoint) {
                    leftCount++;
                }
                else {
                    rightCount++;
                }
                h = mesh.next_halfedge(h);
            } while (h != hf);

            triangleCenter /= vertexCount; // 삼각형의 중심 계산

            // 삼각형이 분할 평면을 가로지르는 경우 양쪽 자식에 추가
            if (leftIntersect && rightIntersect) {
                leftFaces.push_back(f);
                rightFaces.push_back(f);
            }
            else {
                // 삼각형 중심을 기준으로 분류하여 더 균형 잡힌 분할을 유도
                if (triangleCenter[axis] < splitPoint) {
                    leftFaces.push_back(f);
                    if (rightCount > 0) {
                        rightFaces.push_back(f); // 꼭짓점이 오른쪽에도 있으면 오른쪽에도 추가
                    }
                }
                else {
                    rightFaces.push_back(f);
                    if (leftCount > 0) {
                        leftFaces.push_back(f); // 꼭짓점이 왼쪽에도 있으면 왼쪽에도 추가
                    }
                }
            }
        }

        // 분할이 실패했을 경우 이 노드를 리프 노드로 만듦
        if (leftFaces.empty() || rightFaces.empty()) {
            node->face_index = faces; // 모든 면을 현재 노드에 저장
            std::cerr << "Splitting failed at level " << level << "; node becomes a leaf with " << faces.size() << " faces." << std::endl;
            return node;
        }

        // 성공적으로 분할되었다면 재귀적으로 자식 노드 생성
        node->left = buildRecursive(leftFaces, level + 1, maxDepth);
        node->right = buildRecursive(rightFaces, level + 1, maxDepth);

        return node;
    }

    AABB computeAABB(int face_idx) {
        pmp::Face f(face_idx);
        auto hf = mesh.halfedge(f);
        auto h = hf;
        pmp::Vertex v = mesh.to_vertex(h);
        pmp::Point pos = mesh.position(v);
        AABB aabb(pos, pos);

        for (h = mesh.next_halfedge(h); h != hf; h = mesh.next_halfedge(h)) {
            v = mesh.to_vertex(h);
            pos = mesh.position(v);
            aabb = aabb.merge(AABB(pos, pos));
        }
        return aabb;
    }

    void drawAABB(const AABB& box, float color[3]) {

        glColor3f(color[0], color[1], color[2]); // 노드의 색상 설정

        glBegin(GL_LINE_LOOP); // 선분으로 박스를 그립니다
        // 박스의 바닥
        glVertex3f(box.pmin[0], box.pmin[1], box.pmin[2]);
        glVertex3f(box.pmax[0], box.pmin[1], box.pmin[2]);
        glVertex3f(box.pmax[0], box.pmin[1], box.pmax[2]);
        glVertex3f(box.pmin[0], box.pmin[1], box.pmax[2]);
        glEnd();

        glBegin(GL_LINE_LOOP);
        // 박스의 상단
        glVertex3f(box.pmin[0], box.pmax[1], box.pmin[2]);
        glVertex3f(box.pmax[0], box.pmax[1], box.pmin[2]);
        glVertex3f(box.pmax[0], box.pmax[1], box.pmax[2]);
        glVertex3f(box.pmin[0], box.pmax[1], box.pmax[2]);
        glEnd();

        glBegin(GL_LINES);
        // 박스의 수직 선
        glVertex3f(box.pmin[0], box.pmin[1], box.pmin[2]);
        glVertex3f(box.pmin[0], box.pmax[1], box.pmin[2]);

        glVertex3f(box.pmax[0], box.pmin[1], box.pmin[2]);
        glVertex3f(box.pmax[0], box.pmax[1], box.pmin[2]);

        glVertex3f(box.pmax[0], box.pmin[1], box.pmax[2]);
        glVertex3f(box.pmax[0], box.pmax[1], box.pmax[2]);

        glVertex3f(box.pmin[0], box.pmin[1], box.pmax[2]);
        glVertex3f(box.pmin[0], box.pmax[1], box.pmax[2]);
        glEnd();
    }

    void drawRecursive(Node* node, int currentLevel, int arrowNum0, int arrowNum1)
    {
        if (node == nullptr) {
            std::cout << "Node is null" << std::endl;
            return;
        }

        // 자식 노드들을 재귀적으로 그리기
        if (node->left != nullptr) {
            drawRecursive(node->left, currentLevel + 1, arrowNum0, arrowNum1);
        }
        if (node->right != nullptr) {
            drawRecursive(node->right, currentLevel + 1, arrowNum0, arrowNum1);
        }

        // 노드의 색상을 설정하고, 해당 노드의 Face들을 그 색상으로 렌더링
        glColor3f(node->color[0], node->color[1], node->color[2]);

        glBegin(GL_TRIANGLES);
        for (auto& f_idx : node->face_index) {
            pmp::Face f(f_idx);
            auto hf = mesh.halfedge(f);
            auto h = hf;
            do {
                pmp::Vertex v = mesh.to_vertex(h);
                pmp::Point pos = mesh.position(v);
                glVertex3d(pos[0], pos[1], pos[2]);
                h = mesh.next_halfedge(h);
            } while (h != hf);
        }
        glEnd();

        // 노드의 AABB 그리기
        //drawAABB(node->aabb, node->color);

        
    }


    // 노드의 유효성을 검증하는 재귀 함수
    bool validateNode(Node* node, Node* parent)
    {
        // 노드가 nullptr인지 확인
        if (!node) return true;

        // 노드가 최소 1개 이상의 삼각형을 포함하는지 확인
        assert(!node->face_index.empty());

        // 상위 BV가 하위 BV를 포함하는지 확인
        if (parent) {
            assert(parent->aabb.intersect(node->aabb));
        }

        // BV가 자신의 모든 삼각형을 올바르게 포함하는지 확인
        AABB nodeAABB;
        for (auto& face_idx : node->face_index) {
            nodeAABB = nodeAABB.merge(computeAABB(face_idx));
        }
        assert(node->aabb.intersect(nodeAABB));

        // 재귀적으로 자식 노드 검증
        bool leftValid = validateNode(node->left, node);
        bool rightValid = validateNode(node->right, node);

        // 자식 노드의 연결이 정상적인지 확인 (예: 양쪽 자식이 모두 있거나 둘 다 없거나)
        assert((node->left && node->right) || (!node->left && !node->right));

        return leftValid && rightValid;
    }
};
