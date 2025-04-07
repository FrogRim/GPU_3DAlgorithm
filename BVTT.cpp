#include "BVTT.h"
#include "gl/freeglut.h"

// BVTT 클래스의 구현
void BVTT::Build(BVH* bvh1, BVH* bvh2)
{
    this->bvh1 = bvh1;
    this->bvh2 = bvh2;
    BVQueue.push({ bvh1->root, bvh2->root });

    while (!BVQueue.empty()) {
        auto pair = BVQueue.front();
        BVQueue.pop();
        ProcessBVPair(pair.first, pair.second);
    }
}

// 1차 BV 충돌처리 함수
void BVTT::ProcessBVPair(BV* node1, BV* node2)
{

    // node1 또는 node2가 nullptr인 경우 처리하지 않음
    if (!node1 || !node2) return;

    // AABB가 교차하지 않으면 처리하지 않음
    if (!node1->aabb.Intersect(node2->aabb)) return;

    // 두 노드가 모두 리프 노드인 경우
    if (!node1->left && !node1->right && !node2->left && !node2->right) {
        collisionPairs.push_back({ node1, node2 });
        return;
    }

    HandleCollision(node1, node2);
}

// 2차 BV 충돌처리 함수(두 노드가 모두 리프 노드가 아닌 경우)
void BVTT::HandleCollision(BV* node1, BV* node2)
{

    // 두 노드가 모두 리프 노드일 때
    if (!node1->left && !node1->right && !node2->left && !node2->right) {

        // 충돌 리스트에 현재 노드 쌍을 추가
        collisionPairs.push_back({ node1, node2 });
        return;
    }

    // 왼쪽 노드가 리프 노드일 때
    if (!node1->left && !node1->right) {
        // 오른쪽 노드의 자식 노드와 왼쪽 노드를 비교
        if (node2->left) BVQueue.push({ node1, node2->left });
        if (node2->right) BVQueue.push({ node1, node2->right });
        return;
    }

    // 오른쪽 노드가 리프 노드일 때
    if (!node2->left && !node2->right) {
        // 왼쪽 노드의 자식 노드와 오른쪽 노드를 비교
        if (node1->left) BVQueue.push({ node1->left, node2 });
        if (node1->right) BVQueue.push({ node1->right, node2 });
        return;
    }

    // 리프 노드가 아닐 때
    BVQueue.push({ node1->left, node2->left });
    BVQueue.push({ node1->left, node2->right });
    BVQueue.push({ node1->right, node2->left });
    BVQueue.push({ node1->right, node2->right });
}

void BVTT::DrawCollisions()
{
    for (const auto& pair : collisionPairs) {
        DrawAABB(pair.first->aabb);  // 첫 번째 충돌 객체 색칠
        DrawAABB(pair.second->aabb); // 두 번째 충돌 객체 색칠
    }
}

void BVTT::DrawAABB(const AABB& box)
{
    glColor3f(1.0f, 0.0f, 0.0f);

    glBegin(GL_LINE_LOOP);
    glVertex3f(box.pmin[0], box.pmin[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmin[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmin[1], box.pmax[2]);
    glVertex3f(box.pmin[0], box.pmin[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glVertex3f(box.pmin[0], box.pmax[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmax[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmax[1], box.pmax[2]);
    glVertex3f(box.pmin[0], box.pmax[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINES);
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