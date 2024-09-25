#pragma once
#include "pmp/surface_mesh.h"
#include <vector>
#include "BV.h"

class BVH
{
public:
    explicit BVH(pmp::SurfaceMesh& mesh) : mesh(mesh), root(nullptr) {}
    void Build();
    void Draw(int arrowNum0, int arrowNum1);
    void Validate();
    BV* root;

private:
    BV* BuildRecursive(const std::vector<int>& faces, int level);
    void SetColor(BV* BV, int level);
    AABB ComputeAABB(int face_idx);
    void DrawAABB(const AABB& box, float color[3]);
    void DrawRecursive(BV* BV, int currentLevel, int arrowNum0, int arrowNum1);
    bool ValidateBV(BV* node, BV* parent);

    pmp::SurfaceMesh& mesh;

};