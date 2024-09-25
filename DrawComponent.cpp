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

    // 두 번째 모델을 x축으로 0.5 이동
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

    // BVH 구조의 검증 시작
    bvh1->Validate();
    bvh2->Validate();
    cout << "done\n";

    cout << "Build BVTT...";
    bvtt = new BVTT();
    bvtt->Build(bvh1, bvh2);  // 충돌 탐지 구조 구축
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
        bvtt->DrawCollisions();  // 충돌한 부분 색칠
    }
}

void DrawComponent::CheckBVTT()
{
    auto start = std::chrono::high_resolution_clock::now(); // 시간측정 시작

    bvtt->Build(bvh1, bvh2); // BVTT사용

    auto end = std::chrono::high_resolution_clock::now(); // 시간측정 끝
    std::chrono::duration<double> duration = end - start;

    printf("BVTT 방식 시간: %.5f초\n", duration.count());
}

void DrawComponent::CheckBF(){

    auto start = std::chrono::high_resolution_clock::now(); // 시간측정 시작

    std::vector<std::pair<BV*, BV*>> collisionPairs;

    int count = 0;

    // 모든 리프 노드 쌍을 탐색 후 리프 노드 리스트 반환
    for (auto node1 : GetLeafBVs(bvh1->root)) {
        for (auto node2 : GetLeafBVs(bvh2->root)) {
            if (node1->aabb.Intersect(node2->aabb)) {
                collisionPairs.push_back({ node1, node2 });
            }
        }
    }

    auto end = std::chrono::high_resolution_clock::now(); // 시간측정 끝       
    std::chrono::duration<double> duration = end - start;
    printf("BF 방식 시간: %.5f초\n", duration.count());

}

std::vector<BV*> DrawComponent::GetLeafBVs(BV* node)
{
    std::vector<BV*> leafBVs;

    if (!node)
        return leafBVs;
    
    // 현재 노드가 리프노드이면 벡터에 추가
    if (!node->left && !node->right) {
        leafBVs.push_back(node);
    }

    // 현재 노드가 리프노드가 아니라면 재귀를 통헤 리프노드가 나올때까지 탐색
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
