#include "BVH.h"
#include "gl/freeglut.h"
#include <vector>
#include <iostream>

void BVH::Build()
{
    std::vector<int> faces;
    for (auto f : mesh.faces()) {
        faces.push_back(f.idx());
    }
    root = BuildRecursive(faces, 0);

    printf("BVH build complete.\n");
}

void BVH::Draw(int arrowNum0, int arrowNum1) // BVH를 시각화하기 위해 그림
{
    DrawRecursive(root, 0, arrowNum0, arrowNum1); // 루트에서 시작하여 재귀적으로 그림
}
void BVH::Validate()
{
    assert(ValidateBV(root, nullptr)); // 루트 노드부터 검증 시작
}

BV* BVH::BuildRecursive(const std::vector<int>& faces, int level) {

    if (faces.empty()) {
        /* return nullptr;*/
        BV* node = new BV;
        node->level = level;
        return node;
    }

    BV *node = new BV;
    node->level = level;
    node->face_index = faces;
    SetColor(node, level); // 색상 설정

    if (faces.size() == 1) {
        node->aabb = ComputeAABB(faces[0]);
        return node;
    }

    for (auto f : node->face_index) { // 모든 면의 AABB를 계산하여 병합 
        node->aabb = node->aabb.Merge(ComputeAABB(f));
    }

    // AABB를 기준으로 면 분할
    AABB leftAABB, rightAABB;
    std::vector<int> leftFaces, rightFaces;
    node->aabb.Split(leftAABB, rightAABB);

    for (auto f : faces) {
        AABB faceAABB = ComputeAABB(f);
        bool leftIntersect = faceAABB.Intersect(leftAABB);
        bool rightIntersect = faceAABB.Intersect(rightAABB);

        if (leftIntersect)
            leftFaces.push_back(f);

        if (rightIntersect)
            rightFaces.push_back(f);
    }

    /* AABB update. */
    for (auto f : leftFaces)
        leftAABB = leftAABB.Merge(ComputeAABB(f));

    for (auto f : rightFaces)
        rightAABB = rightAABB.Merge(ComputeAABB(f));

    // 분할 실패 또는 분할 결과가 이전과 동일한 경우 더 이상 가장 긴 축을 2등분 하는 것만으로는 분할이 불가능하다고 판단, 리프 노드로 처리
    if (leftFaces.empty() || rightFaces.empty() || (leftFaces.size() == faces.size() || rightFaces.size() == faces.size()))
        return node;

    node->left = BuildRecursive(leftFaces, level + 1);
    node->right = BuildRecursive(rightFaces, level + 1);

    return node;
}

void BVH::SetColor(BV* node, int level) {
    // 색상 설정 로직
    node->color[0] = static_cast<float>((level * 40) % 255) / 255.0f;
    node->color[1] = static_cast<float>((level * 70) % 255) / 255.0f;
    node->color[2] = static_cast<float>((level * 90) % 255) / 255.0f;
}

AABB BVH::ComputeAABB(int face_idx) {
    pmp::Face f(face_idx);
    AABB aabb;

    for (auto v : mesh.vertices(f)) {
	pmp::Point pos = mesh.position(v);
	aabb = aabb.Merge(AABB(pos, pos));
    }

    return aabb;
}

void BVH::DrawAABB(const AABB& box, float color[3]) {

    glColor3f(color[0], color[1], color[2]); // 노드의 색상 설정

    glBegin(GL_LINE_LOOP); // 선분으로 박스를 그립니다
    // 박스의 바닥
    glVertex3d(box.pmin[0], box.pmin[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmin[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmin[1], box.pmax[2]);
    glVertex3d(box.pmin[0], box.pmin[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    // 박스의 상단
    glVertex3d(box.pmin[0], box.pmax[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmax[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmax[1], box.pmax[2]);
    glVertex3d(box.pmin[0], box.pmax[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINES);
    // 박스의 수직 선
    glVertex3d(box.pmin[0], box.pmin[1], box.pmin[2]);
    glVertex3d(box.pmin[0], box.pmax[1], box.pmin[2]);
             
    glVertex3d(box.pmax[0], box.pmin[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmax[1], box.pmin[2]);
             
    glVertex3d(box.pmax[0], box.pmin[1], box.pmax[2]);
    glVertex3d(box.pmax[0], box.pmax[1], box.pmax[2]);
             
    glVertex3d(box.pmin[0], box.pmin[1], box.pmax[2]);
    glVertex3d(box.pmin[0], box.pmax[1], box.pmax[2]);
    glEnd();
}

void BVH::DrawRecursive(BV* node, int currentLevel, int arrowNum0, int arrowNum1)
{
    if (node == nullptr) {
        std::cout << "BV is null" << std::endl;
        return;
    }

    // 노드 레벨을 기반으로 고유 색상 생성
    float r = static_cast<float>((currentLevel * 40) % 255) / 255.0f;
    float g = static_cast<float>((currentLevel * 70) % 255) / 255.0f;
    float b = static_cast<float>((currentLevel * 90) % 255) / 255.0f;

    glColor3f(r, g, b);  // 레벨 기반 고유 색상 설정

    // 노드의 AABB 그리기
    if (arrowNum0 == node->level) {
        DrawAABB(node->aabb, node->color);
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

    }
    else {
        if (node->left != nullptr) {
            DrawRecursive(node->left, currentLevel + 1, arrowNum0, arrowNum1);
        }
        if (node->right != nullptr) {
            DrawRecursive(node->right, currentLevel + 1, arrowNum0, arrowNum1);
        }
    }
}



// 노드의 유효성을 검증하는 재귀 함수
bool BVH::ValidateBV(BV* node, BV* parent)
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
        std::cerr << "Validation error: BV contains no faces at level "
            << node->level << std::endl;
        return false;
    }

    // 상위 BV가 하위 BV를 포함하는지 확인
    if (parent && !parent->aabb.Intersect(node->aabb)) {
        std::cerr << "Validation error: Parent AABB does not intersect with child AABB at level "
            << node->level << std::endl;
        std::cerr << "Parent AABB: Min(" << parent->aabb.pmin << ") Max("
            << parent->aabb.pmax << ")" << std::endl;
        std::cerr << "Child AABB: Min(" << node->aabb.pmin << ") Max("
            << node->aabb.pmax << ")" << std::endl;

        return false;
    }

    // BV가 자신의 모든 삼각형을 올바르게 포함하는지 확인
    AABB nodeAABB;
    for (auto& face_idx : node->face_index) {
        nodeAABB = nodeAABB.Merge(ComputeAABB(face_idx));
    }
    if (!node->aabb.Intersect(nodeAABB)) {
        std::cerr << "Validation error: BV AABB does not correctly encompass its faces' AABB at level "
            << node->level << std::endl;
        return false;
    }

    // 재귀적으로 자식 노드 검증
    bool leftValid = true, rightValid = true;
    if (node->left) leftValid = ValidateBV(node->left, node);
    if (node->right) rightValid = ValidateBV(node->right, node);

    // 자식 노드의 연결이 정상적인지 확인
    if ((node->left && !node->right) || (!node->left && node->right)) {
        std::cerr << "Validation error: BV has an unbalanced number of children at level "
            << node->level << std::endl;
        return false;
    }



    return leftValid && rightValid;
}
