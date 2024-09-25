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

void BVH::Draw(int arrowNum0, int arrowNum1) // BVH�� �ð�ȭ�ϱ� ���� �׸�
{
    DrawRecursive(root, 0, arrowNum0, arrowNum1); // ��Ʈ���� �����Ͽ� ��������� �׸�
}
void BVH::Validate()
{
    assert(ValidateBV(root, nullptr)); // ��Ʈ ������ ���� ����
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
    SetColor(node, level); // ���� ����

    if (faces.size() == 1) {
        node->aabb = ComputeAABB(faces[0]);
        return node;
    }

    for (auto f : node->face_index) { // ��� ���� AABB�� ����Ͽ� ���� 
        node->aabb = node->aabb.Merge(ComputeAABB(f));
    }

    // AABB�� �������� �� ����
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

    // ���� ���� �Ǵ� ���� ����� ������ ������ ��� �� �̻� ���� �� ���� 2��� �ϴ� �͸����δ� ������ �Ұ����ϴٰ� �Ǵ�, ���� ���� ó��
    if (leftFaces.empty() || rightFaces.empty() || (leftFaces.size() == faces.size() || rightFaces.size() == faces.size()))
        return node;

    node->left = BuildRecursive(leftFaces, level + 1);
    node->right = BuildRecursive(rightFaces, level + 1);

    return node;
}

void BVH::SetColor(BV* node, int level) {
    // ���� ���� ����
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

    glColor3f(color[0], color[1], color[2]); // ����� ���� ����

    glBegin(GL_LINE_LOOP); // �������� �ڽ��� �׸��ϴ�
    // �ڽ��� �ٴ�
    glVertex3d(box.pmin[0], box.pmin[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmin[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmin[1], box.pmax[2]);
    glVertex3d(box.pmin[0], box.pmin[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    // �ڽ��� ���
    glVertex3d(box.pmin[0], box.pmax[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmax[1], box.pmin[2]);
    glVertex3d(box.pmax[0], box.pmax[1], box.pmax[2]);
    glVertex3d(box.pmin[0], box.pmax[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINES);
    // �ڽ��� ���� ��
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

    // ��� ������ ������� ���� ���� ����
    float r = static_cast<float>((currentLevel * 40) % 255) / 255.0f;
    float g = static_cast<float>((currentLevel * 70) % 255) / 255.0f;
    float b = static_cast<float>((currentLevel * 90) % 255) / 255.0f;

    glColor3f(r, g, b);  // ���� ��� ���� ���� ����

    // ����� AABB �׸���
    if (arrowNum0 == node->level) {
        DrawAABB(node->aabb, node->color);
    }


    if (node->left == nullptr && node->right == nullptr) // ���� ����� ���,
    {
        glBegin(GL_TRIANGLES); // ���� ����� ��� �׸���
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



// ����� ��ȿ���� �����ϴ� ��� �Լ�
bool BVH::ValidateBV(BV* node, BV* parent)
{

    // ��尡 nullptr���� Ȯ��
    if (!node) {

        return true;
    }

    // ��Ʈ ����� ��� parent�� nullptr�̹Ƿ� Ư���� ó���� �ʿ� ����
    // �׷��� ��Ʈ ��忡�� �ﰢ���� ���ԵǾ�� ��
    if (!parent && node->face_index.empty()) {
        std::cerr << "Validation error: Root node contains no faces." << std::endl;
        return false;
    }

    // ��尡 �ﰢ���� �����ϰ� �ִ��� Ȯ��
    if (node->face_index.empty()) {
        std::cerr << "Validation error: BV contains no faces at level "
            << node->level << std::endl;
        return false;
    }

    // ���� BV�� ���� BV�� �����ϴ��� Ȯ��
    if (parent && !parent->aabb.Intersect(node->aabb)) {
        std::cerr << "Validation error: Parent AABB does not intersect with child AABB at level "
            << node->level << std::endl;
        std::cerr << "Parent AABB: Min(" << parent->aabb.pmin << ") Max("
            << parent->aabb.pmax << ")" << std::endl;
        std::cerr << "Child AABB: Min(" << node->aabb.pmin << ") Max("
            << node->aabb.pmax << ")" << std::endl;

        return false;
    }

    // BV�� �ڽ��� ��� �ﰢ���� �ùٸ��� �����ϴ��� Ȯ��
    AABB nodeAABB;
    for (auto& face_idx : node->face_index) {
        nodeAABB = nodeAABB.Merge(ComputeAABB(face_idx));
    }
    if (!node->aabb.Intersect(nodeAABB)) {
        std::cerr << "Validation error: BV AABB does not correctly encompass its faces' AABB at level "
            << node->level << std::endl;
        return false;
    }

    // ��������� �ڽ� ��� ����
    bool leftValid = true, rightValid = true;
    if (node->left) leftValid = ValidateBV(node->left, node);
    if (node->right) rightValid = ValidateBV(node->right, node);

    // �ڽ� ����� ������ ���������� Ȯ��
    if ((node->left && !node->right) || (!node->left && node->right)) {
        std::cerr << "Validation error: BV has an unbalanced number of children at level "
            << node->level << std::endl;
        return false;
    }



    return leftValid && rightValid;
}
