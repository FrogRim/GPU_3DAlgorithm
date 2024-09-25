#include "BVTT.h"
#include "gl/freeglut.h"

// BVTT Ŭ������ ����
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

// 1�� BV �浹ó�� �Լ�
void BVTT::ProcessBVPair(BV* node1, BV* node2)
{

    // node1 �Ǵ� node2�� nullptr�� ��� ó������ ����
    if (!node1 || !node2) return;

    // AABB�� �������� ������ ó������ ����
    if (!node1->aabb.Intersect(node2->aabb)) return;

    // �� ��尡 ��� ���� ����� ���
    if (!node1->left && !node1->right && !node2->left && !node2->right) {
        collisionPairs.push_back({ node1, node2 });
        return;
    }

    HandleCollision(node1, node2);
}

// 2�� BV �浹ó�� �Լ�(�� ��尡 ��� ���� ��尡 �ƴ� ���)
void BVTT::HandleCollision(BV* node1, BV* node2)
{

    // �� ��尡 ��� ���� ����� ��
    if (!node1->left && !node1->right && !node2->left && !node2->right) {

        // �浹 ����Ʈ�� ���� ��� ���� �߰�
        collisionPairs.push_back({ node1, node2 });
        return;
    }

    // ���� ��尡 ���� ����� ��
    if (!node1->left && !node1->right) {
        // ������ ����� �ڽ� ���� ���� ��带 ��
        if (node2->left) BVQueue.push({ node1, node2->left });
        if (node2->right) BVQueue.push({ node1, node2->right });
        return;
    }

    // ������ ��尡 ���� ����� ��
    if (!node2->left && !node2->right) {
        // ���� ����� �ڽ� ���� ������ ��带 ��
        if (node1->left) BVQueue.push({ node1->left, node2 });
        if (node1->right) BVQueue.push({ node1->right, node2 });
        return;
    }

    // ���� ��尡 �ƴ� ��
    BVQueue.push({ node1->left, node2->left });
    BVQueue.push({ node1->left, node2->right });
    BVQueue.push({ node1->right, node2->left });
    BVQueue.push({ node1->right, node2->right });
}

void BVTT::DrawCollisions()
{
    for (const auto& pair : collisionPairs) {
        DrawAABB(pair.first->aabb);  // ù ��° �浹 ��ü ��ĥ
        DrawAABB(pair.second->aabb); // �� ��° �浹 ��ü ��ĥ
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