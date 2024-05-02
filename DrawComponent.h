//���
#pragma once
#include "pmp/surface_mesh.h"
#include "gl/freeglut.h"
#include <cassert> 
#include <stack>



// ������ ����
class BVH;

class DrawComponent
{
public:
    void Init();

    void Draw();



    pmp::SurfaceMesh mesh;

    BVH* bvh;  // BVH ��ü ������

    int arrowNum0 = 0;
    int arrowNum1 = 0;

    ~DrawComponent() {  // �Ҹ���
        delete bvh;  // BVH ��ü ����
    }
};

class AABB
{
public:
    AABB() {} // �⺻ ������
    AABB(pmp::Point pmin, pmp::Point pmax) : pmin(pmin), pmax(pmax) {} // �ּ����� �ִ������� �ʱ�ȭ

    pmp::Point pmin; // �ּ� ��ǥ
    pmp::Point pmax; // �ִ� ��ǥ

    // �ٸ� AABB���� ���� ���� �Ǵ�
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

    // Ư�� ������ ���� ���� �Ǵ�
    bool intersect(const pmp::Point& p)
    {
        if (p[0] < pmin[0] || p[0] > pmax[0]) return false;
        if (p[1] < pmin[1] || p[1] > pmax[1]) return false;
        if (p[2] < pmin[2] || p[2] > pmax[2]) return false;
        return true;
    }

    // Ư�� ������ ���� ���͸� ���� ���� ���� �Ǵ�
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

    // �ٸ� AABB�� ��ħ
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

    // ���� �� �� ��ȯ
    int longestAxis()
    {
        pmp::Point d = pmax - pmin;
        if (d[0] > d[1] && d[0] > d[2]) return 0;
        if (d[1] > d[2]) return 1;
        return 2;
    }

    // �߽��� ��ȯ
    pmp::Point center()
    {
        return 0.5 * (pmin + pmax);
    }

    // ��带 �� ���� ����
    void split(AABB& left, AABB& right)
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

    
};

class Node {
public:
    AABB aabb; // AABB ��ü
    std::vector<pmp::Face> face_index; // �� �ε��� ���
    Node* left; // ���� �ڽ� ���
    Node* right; // ������ �ڽ� ���
    int level; // ��� ����

    Node() : left(nullptr), right(nullptr) {} // �⺻ ������
};

class BVH
{
public:
    int cnt = 0;
    explicit BVH(pmp::SurfaceMesh& mesh) : mesh(mesh), root(nullptr) {} // ������: �޽� ������ BVH �ʱ�ȭ
    int maxDepth = 128; // ��� ���� ���Ѽ��� -> �����÷ο� ����
    pmp::SurfaceMesh& mesh; // �޽� ����
    Node* root; // ��Ʈ ���

    void build()
    {
        std::vector<pmp::Face> faces(mesh.faces().begin(), mesh.faces().end());
        root = buildRecursive(faces, 0, maxDepth);
        std::cout << "BVH build complete." << std::endl;
    }

    void draw(int arrowNum0, int arrowNum1) // BVH�� �ð�ȭ�ϱ� ���� �׸�
    {
        drawRecursive(root, 0, arrowNum0, arrowNum1); // ��Ʈ���� �����Ͽ� ��������� �׸�
    }
    void validate()
    {
        assert(validateNode(root, nullptr)); // ��Ʈ ������ ���� ����
    }
private:
    Node* buildRecursive(const std::vector<pmp::Face>& faces, int level, int maxDepth) {
       
        if (level > maxDepth || faces.empty()) {
            Node* node = new Node;
            node->level = level;
            node->face_index = faces; // ������� �ʴٸ�, ���� ����� ����
            return node;
        }

        Node* node = new Node;
        node->level = level;

        // ��� ���� AABB ���
        AABB aabb;
        for (auto f : faces) {
            aabb = aabb.merge(computeAABB(f));
        }
        node->aabb = aabb;

        if (faces.size() <= 1) {  // ���� ��� ����
            node->face_index = faces;
            std::cout << "Leaf node created at level " << level << " with 1 face." << std::endl;
            return node;
        }

        // ���� �¿� �ڽ� ���� ����
        AABB leftAABB, rightAABB;
        std::vector<pmp::Face> leftFaces, rightFaces;
        aabb.split(leftAABB, rightAABB);

        for (auto f : faces) {
            AABB faceAABB = computeAABB(f);
            bool leftIntersect = faceAABB.intersect(leftAABB);
            bool rightIntersect = faceAABB.intersect(rightAABB);

            int axis = aabb.longestAxis();
            double splitPoint = aabb.center()[axis];

            int leftCount = 0, rightCount = 0;
            auto hf = mesh.halfedge(f);
            auto h = hf;
            pmp::Point triangleCenter(0.0, 0.0, 0.0);
            int vertexCount = 0;

            // �ﰢ���� �� ������ ��ġ�� �����Ͽ� ���� ��Ģ�� ����
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

            triangleCenter /= vertexCount; // �ﰢ���� �߽� ���

            // �ﰢ���� ���� ����� ���������� ��� ���� �ڽĿ� �߰�
            if (leftIntersect && rightIntersect) {
                leftFaces.push_back(f);
                rightFaces.push_back(f);
            }
            else {
                // �ﰢ�� �߽��� �������� �з��Ͽ� �� ���� ���� ������ ����
                if (triangleCenter[axis] < splitPoint) {
                    leftFaces.push_back(f);
                    if (rightCount > 0) {
                        rightFaces.push_back(f); // �������� �����ʿ��� ������ �����ʿ��� �߰�
                    }
                }
                else {
                    rightFaces.push_back(f);
                    if (leftCount > 0) {
                        leftFaces.push_back(f); // �������� ���ʿ��� ������ ���ʿ��� �߰�
                    }
                }
            }

			
        }

        // ������ �������� ��� �� ��带 ���� ���� ����
        if (leftFaces.empty() || rightFaces.empty()) {
            node->face_index = faces; // ��� ���� ���� ��忡 ����
            std::cerr << "Splitting failed at level " << level << "; node becomes a leaf with " << faces.size() << " faces." << std::endl;
            return node;
        }

        // ���������� ���ҵǾ��ٸ� ��������� �ڽ� ��� ����
        node->left = buildRecursive(leftFaces, level + 1, maxDepth);
        node->right = buildRecursive(rightFaces, level + 1, maxDepth);

        return node;
    }



    AABB computeAABB(pmp::Face f) {
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
    void drawAABB(const AABB& box,int cnt) {

        if (cnt == 0)
		{
			glColor3f(1.0f, 0.0f, 0.0f); // ������ ����
		}
		else if (cnt == 1)
		{
			glColor3f(0.0f, 0.0f, 1.0f); // �Ķ��� ����
		}
		else
		{
            glColor3f(0.0f, 1.0f, 0.0f); // �ʷϻ� ����
		}
        
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

    void drawRecursive(Node* node, int currentLevel, int arrowNum0, int arrowNum1)
    {
		int cnt = node->level;
        if (node == nullptr) {

            std::cout << "not node" << std::endl;
            return;
        }
        // ��� ������ ����
        if (currentLevel == arrowNum0) {
            glColor3f(1.0, 0.0, 0.0);  // ������ arrowNum0�� �ش��ϸ� ������
        }
        else if (currentLevel == arrowNum1) {
            glColor3f(0.0, 0.0, 1.0);  // ������ arrowNum1�� �ش��ϸ� �Ķ���
        }
        else {
            glColor3f(0.5, 0.5, 0.5);  // �� ���� ������ ȸ��
        }

        if (node->left == nullptr && node->right == nullptr) // ���� ����� ���,
        {
            // ��� ���� ��忡�� �׷����� �׸�
            for (auto& f : node->face_index) {
               
                glBegin(GL_TRIANGLES);
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
            std::cout << "Rendering at leaf node, level " << currentLevel << std::endl;
        }
        else {

			drawAABB(node->aabb,cnt+1);  // ����� AABB�� �׸�
            if (node->left != nullptr) {
                drawRecursive(node->left, currentLevel + 1, arrowNum0, arrowNum1);  // ���� �ڽ� ��� ��������� �׸�
            }
            if (node->right != nullptr) {
                drawRecursive(node->right, currentLevel + 1, arrowNum0, arrowNum1); // ������ �ڽ� ��� ��������� �׸�
            }
			std::cout << "Rendering at internal node, level " << currentLevel <<", " << arrowNum0 << " ," << arrowNum1 << std::endl;
        }

    }

    // ����� ��ȿ���� �����ϴ� ��� �Լ�
    bool validateNode(Node* node, Node* parent)
    {
        // ��尡 nullptr���� Ȯ��
        if (!node) return true;

        // ��尡 �ּ� 1�� �̻��� �ﰢ���� �����ϴ��� Ȯ��
        assert(!node->face_index.empty());

        // ���� BV�� ���� BV�� �����ϴ��� Ȯ��
        if (parent) {
            assert(parent->aabb.intersect(node->aabb));
        }

        // BV�� �ڽ��� ��� �ﰢ���� �ùٸ��� �����ϴ��� Ȯ��
        AABB nodeAABB;
        for (auto& face : node->face_index) {
            nodeAABB = nodeAABB.merge(computeAABB(face));
        }
        assert(node->aabb.intersect(nodeAABB));

        // ��������� �ڽ� ��� ����
        bool leftValid = validateNode(node->left, node);
        bool rightValid = validateNode(node->right, node);

        // �ڽ� ����� ������ ���������� Ȯ�� (��: ���� �ڽ��� ��� �ְų� �� �� ���ų�)
        assert((node->left && node->right) || (!node->left && !node->right));

        return leftValid && rightValid;
    }
};