#pragma once
#include "BVH.h"
#include <queue>
#include <vector>
#include "BV.h"
class BVTT
{
public:
    void Build(BVH* bvh1, BVH* bvh2);
    void DrawCollisions();

private:
    void ProcessBVPair(BV* BV1, BV* BV2);
    void HandleCollision(BV* BV1, BV* BV2);
    void DrawAABB(const AABB& box);

    BVH* bvh1 = nullptr;
    BVH* bvh2 = nullptr;
    std::queue<std::pair<BV*, BV*>> BVQueue;  // 큐로 관리
    std::vector<std::pair<BV*, BV*>> collisionPairs;  // 충돌 리스트
};
