#include "DrawComponent.h"
#include "gl/freeglut.h"

#include"pmp/io/io.h"
#include"pmp/algorithms/normals.h"
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;

void DrawComponent::Init() {

    pmp::read(mesh, "obj\\bunny_stanford.obj");
    pmp::vertex_normals(mesh);
    cout << "Loaded mesh with " << mesh.n_vertices() << " vertices and " << mesh.n_faces() << " faces." << endl;


    bvh = new BVH(mesh);
    bvh->build();

    // BVH 구조의 검증 시작
    bvh->validate();

	
}


void DrawComponent::Draw() {

    if (bvh) {

        bvh->draw(arrowNum0, arrowNum1);
        cout << arrowNum0 << endl;
    }
}

// 다른 AABB와의 교차 여부 판단
bool AABB::intersect(const AABB& other)
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
bool AABB::intersect(const pmp::Point& p)
{
    if (p[0] < pmin[0] || p[0] > pmax[0]) return false;
    if (p[1] < pmin[1] || p[1] > pmax[1]) return false;
    if (p[2] < pmin[2] || p[2] > pmax[2]) return false;
    return true;
}

// 특정 점에서 방향 벡터를 따라 교차 여부 판단
bool AABB::intersect(const pmp::Point& p, const pmp::Point& d)
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
AABB AABB::merge(const AABB& other)
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
int AABB::longestAxis()
{
    pmp::Point d = pmax - pmin;
    if (d[0] > d[1] && d[0] > d[2]) return 0;
    if (d[1] > d[2]) return 1;
    return 2;
}

// 중심점 반환
pmp::Point AABB::center()
{
    return 0.5 * (pmin + pmax);
}

// 노드를 두 개로 분할
void AABB::split(AABB& left, AABB& right)
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

void BVH::build()
{
    std::vector<int> faces;
    for (auto f : mesh.faces()) {
        faces.push_back(f.idx());
    }
    root = buildRecursive(faces, 0);
    std::cout << "BVH build complete." << std::endl;
}

void BVH::draw(int arrowNum0, int arrowNum1) // BVH를 시각화하기 위해 그림
{
    drawRecursive(root, 0, arrowNum0, arrowNum1); // 루트에서 시작하여 재귀적으로 그림
}
void BVH::validate()
{
    assert(validateNode(root, nullptr)); // 루트 노드부터 검증 시작
}

Node* BVH::buildRecursive(const std::vector<int>& faces, int level) {

    if (faces.empty()) {
        Node* node = new Node;
        node->level = level;
        return node;
    }

    if (faces.size() <= 1) {
        Node* node = new Node;
        node->level = level;
        node->face_index = faces;

        setColor(node, level); // 색상 설정

        AABB tempAABB = computeAABB(faces[0]);
        node->aabb.pmax = tempAABB.pmax;
        node->aabb.pmin = tempAABB.pmin;

        return node;
    }

    Node* node = new Node;
    node->level = level;
	node->face_index = faces;

    setColor(node, level); // 색상 설정

    AABB aabb;
	for (auto f : node->face_index) { // 모든 면의 AABB를 계산하여 병합 
        aabb = aabb.merge(computeAABB(f));
    }
    node->aabb = aabb;

    // AABB를 기준으로 면 분할
    AABB leftAABB, rightAABB;
    std::vector<int> leftFaces, rightFaces;
    aabb.split(leftAABB, rightAABB);

    for (auto f : faces) {
        AABB faceAABB = computeAABB(f);
        bool leftIntersect = faceAABB.intersect(leftAABB);
        bool rightIntersect = faceAABB.intersect(rightAABB);


        if (leftIntersect) {
            leftFaces.push_back(f);
        }
        if (rightIntersect) {
            rightFaces.push_back(f);
        }
    }
    //std::cout << "Level: " << level << ", Left count: " << leftFaces.size() << ", Right count: " << rightFaces.size() << std::endl;

    // 분할 실패 또는 분할 결과가 이전과 동일한 경우 더 이상 가장 긴 축을 2등분 하는 것만으로는 분할이 불가능하다고 판단, 리프 노드로 처리
    if (leftFaces.empty() || rightFaces.empty() || (leftFaces.size() == faces.size() || rightFaces.size() == faces.size())) {
        node->face_index = faces;
        //std::cerr << "Splitting failed at level " << level << "; node becomes a leaf with " << faces.size() << " faces." << std::endl;
        return node;
    }

    node->left = buildRecursive(leftFaces, level + 1);
    node->right = buildRecursive(rightFaces, level + 1);

    return node;
}

void BVH::setColor(Node* node, int level) {
    // 색상 설정 로직
    node->color[0] = static_cast<float>((level * 40) % 255) / 255.0f;
    node->color[1] = static_cast<float>((level * 70) % 255) / 255.0f;
    node->color[2] = static_cast<float>((level * 90) % 255) / 255.0f;
}

AABB BVH::computeAABB(int face_idx) {
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

void BVH::drawAABB(const AABB& box, float color[3]) {

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

void BVH::drawRecursive(Node* node, int currentLevel, int arrowNum0, int arrowNum1)
{
    if (node == nullptr) {
        std::cout << "Node is null" << std::endl;
        return;
    }

    // 노드 레벨을 기반으로 고유 색상 생성
    float r = static_cast<float>((currentLevel * 40) % 255) / 255.0f;
    float g = static_cast<float>((currentLevel * 70) % 255) / 255.0f;
    float b = static_cast<float>((currentLevel * 90) % 255) / 255.0f;

    glColor3f(r, g, b);  // 레벨 기반 고유 색상 설정

    // 노드의 AABB 그리기
	if (arrowNum0 == node->level) {
		drawAABB(node->aabb, node->color);
	}
   

    if (node->left == nullptr && node->right == nullptr) // 리프 노드인 경우,
    {
        glBegin(GL_TRIANGLES); // 리프 노드의 면들 그리기
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
        //std::cout << "Rendering at leaf node, level " << currentLevel << std::endl;
    }
    else {
        if (node->left != nullptr) {
            drawRecursive(node->left, currentLevel + 1, arrowNum0, arrowNum1);  // 왼쪽 자식 노드 재귀적으로 그리기
        }
        if (node->right != nullptr) {
            drawRecursive(node->right, currentLevel + 1, arrowNum0, arrowNum1); // 오른쪽 자식 노드 재귀적으로 그리기
        }
    }
}



// 노드의 유효성을 검증하는 재귀 함수
bool BVH::validateNode(Node* node, Node* parent)
{
	
    // 노드가 nullptr인지 확인
    if (!node) {
        
        return true;
    }

    // 루트 노드의 경우 parent가 nullptr이므로 특별한 처리가 필요 없음
    // 그러나 루트 노드에도 삼각형이 포함되어야 함
    if (!parent && node->face_index.empty()) {
        std::cerr << "Validation error: Root node contains no faces." << std::endl;
        return false;
    }

    // 노드가 삼각형을 포함하고 있는지 확인
    if (node->face_index.empty()) {
        std::cerr << "Validation error: Node contains no faces at level " << node->level << std::endl;
        return false;
    }

    // 상위 BV가 하위 BV를 포함하는지 확인
    if (parent && !parent->aabb.intersect(node->aabb)) {
        std::cerr << "Validation error: Parent AABB does not intersect with child AABB at level " << node->level << std::endl;
        std::cerr << "Parent AABB: Min(" << parent->aabb.pmin << ") Max(" << parent->aabb.pmax << ")" << std::endl;
        std::cerr << "Child AABB: Min(" << node->aabb.pmin << ") Max(" << node->aabb.pmax << ")" << std::endl;
        
        return false;
    }

    // BV가 자신의 모든 삼각형을 올바르게 포함하는지 확인
    AABB nodeAABB;
    for (auto& face_idx : node->face_index) {
        nodeAABB = nodeAABB.merge(computeAABB(face_idx));
    }
    if (!node->aabb.intersect(nodeAABB)) {
        std::cerr << "Validation error: Node AABB does not correctly encompass its faces' AABB at level " << node->level << std::endl;
        return false;
    }

    // 재귀적으로 자식 노드 검증
    bool leftValid = true, rightValid = true;
    if (node->left) leftValid = validateNode(node->left, node);
    if (node->right) rightValid = validateNode(node->right, node);

    // 자식 노드의 연결이 정상적인지 확인
    if ((node->left && !node->right) || (!node->left && node->right)) {
        std::cerr << "Validation error: Node has an unbalanced number of children at level " << node->level << std::endl;
        return false;
    }

    //std::cout << "Node at level " << node->level << " passed validation." << std::endl;

    return leftValid && rightValid;
}
