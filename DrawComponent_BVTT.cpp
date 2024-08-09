#include "DrawComponent.h"
#include "gl/freeglut.h"

#include"pmp/io/io.h"
#include"pmp/algorithms/normals.h"
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;

void DrawComponent::Init() {

    pmp::read(mesh1, "obj\\bunny_stanford.obj");
    pmp::vertex_normals(mesh1);
    std::cout << "Loaded first mesh with " << mesh1.n_vertices() << " vertices and " << mesh1.n_faces() << " faces." << std::endl;

    pmp::read(mesh2, "obj\\bunny_stanford.obj");
    pmp::vertex_normals(mesh2);
    std::cout << "Loaded second mesh with " << mesh2.n_vertices() << " vertices and " << mesh2.n_faces() << " faces." << std::endl;

	auto points = mesh1.vertex_property<pmp::Point>("v:point");

    // �� ��° ���� x������ 0.5 �̵�
    for (auto v : mesh2.vertices()) {
        auto pos = mesh2.position(v);
        pos[0] += 0.5;
		points[v] = pos;
    }

    bvh1 = new BVH(mesh1);
    bvh1->build();

    bvh2 = new BVH(mesh2);
    bvh2->build();

    // BVH ������ ���� ����
    bvh1->validate();
    bvh2->validate();

    bvtt = new BVTT();
    bvtt->build(bvh1, bvh2);  // �浹 Ž�� ���� ����
}


void DrawComponent::Draw() {

    
    if (bvh1) {
       bvh1->draw(arrowNum0, arrowNum1);
    }
    if (bvh2) {
        bvh2->draw(arrowNum0, arrowNum1);
    }

    if (bvtt) {
        bvtt->drawCollisions();  // �浹�� �κ� ��ĥ
    }
}

// �ٸ� AABB���� ���� ���� �Ǵ�
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

// Ư�� ������ ���� ���� �Ǵ�
bool AABB::intersect(const pmp::Point& p)
{
    if (p[0] < pmin[0] || p[0] > pmax[0]) return false;
    if (p[1] < pmin[1] || p[1] > pmax[1]) return false;
    if (p[2] < pmin[2] || p[2] > pmax[2]) return false;
    return true;
}

// Ư�� ������ ���� ���͸� ���� ���� ���� �Ǵ�
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

// �ٸ� AABB�� ��ħ
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

// ���� �� �� ��ȯ
int AABB::longestAxis()
{
    pmp::Point d = pmax - pmin;
    if (d[0] > d[1] && d[0] > d[2]) return 0;
    if (d[1] > d[2]) return 1;
    return 2;
}

// �߽��� ��ȯ
pmp::Point AABB::center()
{
    return 0.5 * (pmin + pmax);
}

// ��带 �� ���� ����
void AABB::split(AABB& left, AABB& right)
{
    int axis = longestAxis();
    double mid = (pmax[axis] + pmin[axis]) * 0.5;


    pmp::Point pmin_l = pmin;
    pmp::Point pmax_l = pmax;
    pmp::Point pmin_r = pmin;
    pmp::Point pmax_r = pmax;

    pmax_l[axis] = mid; // ���� ������ �ִ� AABB ����
    pmin_r[axis] = mid; // ������ ������ �ּ� AABB ����

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

void BVH::draw(int arrowNum0, int arrowNum1) // BVH�� �ð�ȭ�ϱ� ���� �׸�
{
    drawRecursive(root, 0, arrowNum0, arrowNum1); // ��Ʈ���� �����Ͽ� ��������� �׸�
}
void BVH::validate()
{
    assert(validateNode(root, nullptr)); // ��Ʈ ������ ���� ����
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

        setColor(node, level); // ���� ����

        AABB tempAABB = computeAABB(faces[0]);
        node->aabb.pmax = tempAABB.pmax;
        node->aabb.pmin = tempAABB.pmin;

        return node;
    }

    Node* node = new Node;
    node->level = level;
    node->face_index = faces;

    setColor(node, level); // ���� ����

    AABB aabb;
    for (auto f : node->face_index) { // ��� ���� AABB�� ����Ͽ� ���� 
        aabb = aabb.merge(computeAABB(f));
    }
    node->aabb = aabb;

    // AABB�� �������� �� ����
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

    // ���� ���� �Ǵ� ���� ����� ������ ������ ��� �� �̻� ���� �� ���� 2��� �ϴ� �͸����δ� ������ �Ұ����ϴٰ� �Ǵ�, ���� ���� ó��
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
    // ���� ���� ����
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

    glColor3f(color[0], color[1], color[2]); // ����� ���� ����

    glBegin(GL_LINE_LOOP); // �������� �ڽ��� �׸��ϴ�
    // �ڽ��� �ٴ�
    glVertex3f(box.pmin[0], box.pmin[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmin[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmin[1], box.pmax[2]);
    glVertex3f(box.pmin[0], box.pmin[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINE_LOOP);
    // �ڽ��� ���
    glVertex3f(box.pmin[0], box.pmax[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmax[1], box.pmin[2]);
    glVertex3f(box.pmax[0], box.pmax[1], box.pmax[2]);
    glVertex3f(box.pmin[0], box.pmax[1], box.pmax[2]);
    glEnd();

    glBegin(GL_LINES);
    // �ڽ��� ���� ��
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

    // ��� ������ ������� ���� ���� ����
    float r = static_cast<float>((currentLevel * 40) % 255) / 255.0f;
    float g = static_cast<float>((currentLevel * 70) % 255) / 255.0f;
    float b = static_cast<float>((currentLevel * 90) % 255) / 255.0f;

    glColor3f(r, g, b);  // ���� ��� ���� ���� ����

    // ����� AABB �׸���
    if (arrowNum0 == node->level) {
        drawAABB(node->aabb, node->color);
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
        //std::cout << "Rendering at leaf node, level " << currentLevel << std::endl;
    }
    else {
        if (node->left != nullptr) {
            drawRecursive(node->left, currentLevel + 1, arrowNum0, arrowNum1);  // ���� �ڽ� ��� ��������� �׸���
        }
        if (node->right != nullptr) {
            drawRecursive(node->right, currentLevel + 1, arrowNum0, arrowNum1); // ������ �ڽ� ��� ��������� �׸���
        }
    }
}



// ����� ��ȿ���� �����ϴ� ��� �Լ�
bool BVH::validateNode(Node* node, Node* parent)
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
        std::cerr << "Validation error: Node contains no faces at level " << node->level << std::endl;
        return false;
    }

    // ���� BV�� ���� BV�� �����ϴ��� Ȯ��
    if (parent && !parent->aabb.intersect(node->aabb)) {
        std::cerr << "Validation error: Parent AABB does not intersect with child AABB at level " << node->level << std::endl;
        std::cerr << "Parent AABB: Min(" << parent->aabb.pmin << ") Max(" << parent->aabb.pmax << ")" << std::endl;
        std::cerr << "Child AABB: Min(" << node->aabb.pmin << ") Max(" << node->aabb.pmax << ")" << std::endl;

        return false;
    }

    // BV�� �ڽ��� ��� �ﰢ���� �ùٸ��� �����ϴ��� Ȯ��
    AABB nodeAABB;
    for (auto& face_idx : node->face_index) {
        nodeAABB = nodeAABB.merge(computeAABB(face_idx));
    }
    if (!node->aabb.intersect(nodeAABB)) {
        std::cerr << "Validation error: Node AABB does not correctly encompass its faces' AABB at level " << node->level << std::endl;
        return false;
    }

    // ��������� �ڽ� ��� ����
    bool leftValid = true, rightValid = true;
    if (node->left) leftValid = validateNode(node->left, node);
    if (node->right) rightValid = validateNode(node->right, node);

    // �ڽ� ����� ������ ���������� Ȯ��
    if ((node->left && !node->right) || (!node->left && node->right)) {
        std::cerr << "Validation error: Node has an unbalanced number of children at level " << node->level << std::endl;
        return false;
    }

    //std::cout << "Node at level " << node->level << " passed validation." << std::endl;

    return leftValid && rightValid;
}

// BVTT Ŭ������ ����
void BVTT::build(BVH* bvh1, BVH* bvh2) 
{
    this->bvh1 = bvh1;
    this->bvh2 = bvh2;
    nodeQueue.push({ bvh1->root, bvh2->root });

    while (!nodeQueue.empty()) {
        auto pair = nodeQueue.front();
        nodeQueue.pop();
        processNodePair(pair.first, pair.second);
    }
}

// 1�� BV �浹ó�� �Լ�
void BVTT::processNodePair(Node* node1, Node* node2)
{
    
	// node1 �Ǵ� node2�� nullptr�� ��� ó������ ����
    if (!node1 || !node2) return;

	// AABB�� �������� ������ ó������ ����
    if (!node1->aabb.intersect(node2->aabb)) return;

	// �� ��尡 ��� ���� ����� ���
    if (!node1->left && !node1->right && !node2->left && !node2->right) {
        collisionPairs.push_back({ node1, node2 });
        return;
    }

    handleCollision(node1, node2);
}

// 2�� BV �浹ó�� �Լ�(�� ��尡 ��� ���� ��尡 �ƴ� ���)
void BVTT::handleCollision(Node* node1, Node* node2) 
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
        if (node2->left) nodeQueue.push({ node1, node2->left });
        if (node2->right) nodeQueue.push({ node1, node2->right });
        return;
    }

    // ������ ��尡 ���� ����� ��
    if (!node2->left && !node2->right) {
        // ���� ����� �ڽ� ���� ������ ��带 ��
        if (node1->left) nodeQueue.push({ node1->left, node2 });
        if (node1->right) nodeQueue.push({ node1->right, node2 });
        return;
    }

    // ���� ��尡 �ƴ� ��
    nodeQueue.push({ node1->left, node2->left });
    nodeQueue.push({ node1->left, node2->right });
    nodeQueue.push({ node1->right, node2->left });
    nodeQueue.push({ node1->right, node2->right });
}

void BVTT::drawCollisions() 
{
    for (const auto& pair : collisionPairs) {
        drawAABB(pair.first->aabb);  // ù ��° �浹 ��ü ��ĥ
        drawAABB(pair.second->aabb); // �� ��° �浹 ��ü ��ĥ
    }
}

void BVTT::drawAABB(const AABB& box) const 
{
    glColor3f(1.0f,0.0f,0.0f);

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