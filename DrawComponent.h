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

    class BVH* bvh1 = nullptr;  // BVH ��ü ������
    class BVH* bvh2 = nullptr;  // BVH ��ü ������

    class BVTT* bvtt = nullptr;  // BVTT ��ü ������

    int arrowNum0 = 0;
    int arrowNum1 = 0;

    ~DrawComponent() {  // �Ҹ���
        delete bvh1;  // BVH ��ü ����
        delete bvh2;
        delete bvtt;
    }
};





