#include "DrawComponent.h"
#include "BVH.h"
#include "BVTT.h"
#include "BV.h"
#include "gl/freeglut.h"
#include<chrono>
#include"pmp/io/io.h"
#include"pmp/algorithms/normals.h"
#include <vector>
#include <iostream>


using namespace std;

void DrawComponent::Init() {

    cout << "Read mesh...";
    pmp::read(mesh1, "obj\\icosahedron.obj");
    pmp::vertex_normals(mesh1);

    pmp::read(mesh2, "obj\\icosahedron.obj");
    pmp::vertex_normals(mesh2);

    auto points = mesh1.vertex_property<pmp::Point>("v:point");

    // �� ��° ���� x������ 0.5 �̵�
    for (auto v : mesh2.vertices()) {
        auto pos = mesh2.position(v);
        pos[0] += 1.5;
        points[v] = pos;
    }
    cout << "done\n";

    cout << "Build BVH...";
    bvh1 = new BVH(mesh1);
    bvh1->Build();

    bvh2 = new BVH(mesh2);
    bvh2->Build();

    // BVH ������ ���� ����
    bvh1->Validate();
    bvh2->Validate();
    cout << "done\n";

    cout << "Build BVTT...";
    bvtt = new BVTT();
    bvtt->Build(bvh1, bvh2);  // �浹 Ž�� ���� ����
    cout << "Done\n";

    cout << "Check BVTT...";
    CheckBVTT();
    cout << "Done\n";

    cout << "Check BF...";
    CheckBF();
    cout << "Done\n";
}


void DrawComponent::Draw() {


    if (bvh1) {
        bvh1->Draw(arrowNum0, arrowNum1);
    }
    if (bvh2) {
        bvh2->Draw(arrowNum0, arrowNum1);
    }

    if (bvtt) {
        bvtt->DrawCollisions();  // �浹�� �κ� ��ĥ
    }
}

void DrawComponent::CheckBVTT()
{
    auto start = std::chrono::high_resolution_clock::now(); // �ð����� ����

    bvtt->Build(bvh1, bvh2); // BVTT���

    auto end = std::chrono::high_resolution_clock::now(); // �ð����� ��
    std::chrono::duration<double> duration = end - start;

    printf("BVTT ��� �ð�: %.5f��\n", duration.count());
}

void DrawComponent::CheckBF(){

    auto start = std::chrono::high_resolution_clock::now(); // �ð����� ����

    std::vector<std::pair<BV*, BV*>> collisionPairs;

    int count = 0;

    // ��� ���� ��� ���� Ž�� �� ���� ��� ����Ʈ ��ȯ
    for (auto node1 : GetLeafBVs(bvh1->root)) {
        for (auto node2 : GetLeafBVs(bvh2->root)) {
            if (node1->aabb.Intersect(node2->aabb)) {
                collisionPairs.push_back({ node1, node2 });
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now(); // �ð����� ��       
    std::chrono::duration<double> duration = end - start;
    printf("BF ��� �ð�: %.5f��\n", duration.count());

}

std::vector<BV*> DrawComponent::GetLeafBVs(BV* node)
{
    std::vector<BV*> leafBVs;

    if (!node)
        return leafBVs;
    
    // ���� ��尡 ��������̸� ���Ϳ� �߰�
    if (!node->left && !node->right) {
        leafBVs.push_back(node);
    }

    // ���� ��尡 ������尡 �ƴ϶�� ��͸� ���� ������尡 ���ö����� Ž��
    else {
        if (node->left) {
            auto leftLeaves = GetLeafBVs(node->left);
            leafBVs.insert(leafBVs.end(), leftLeaves.begin(), leftLeaves.end());
        }
        if (node->right) {
            auto rightLeaves = GetLeafBVs(node->right);
            leafBVs.insert(leafBVs.end(), rightLeaves.begin(), rightLeaves.end());
        }
    }


    return leafBVs;

}
