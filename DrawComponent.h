#pragma once

#include "pmp/surface_mesh.h"
#include <vector>
#include "BVH.h"
#include "BVTT.h"


class DrawComponent
{
public:
    void Init();
    void Draw();
    void CheckBVTT();
    void CheckBF();
    std::vector<BV*> GetLeafBVs(BV* BV);

    pmp::SurfaceMesh mesh1;
    pmp::SurfaceMesh mesh2;

    class BVH* bvh1 = nullptr;  // BVH 객체 포인터
    class BVH* bvh2 = nullptr;  // BVH 객체 포인터

    class BVTT* bvtt = nullptr;  // BVTT 객체 포인터

    int arrowNum0 = 0;
    int arrowNum1 = 0;

    ~DrawComponent() {  // 소멸자
        delete bvh1;  // BVH 객체 삭제
        delete bvh2;
        delete bvtt;
    }
};





