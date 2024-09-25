#pragma once
#include"pmp/surface_mesh.h"
#include <vector>

class AABB
{
public:
    AABB() // 기본 생성자
    {
        pmin = pmp::Point(std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        pmax = pmp::Point(std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());
    }
    AABB(pmp::Point pmin, pmp::Point pmax) : pmin(pmin), pmax(pmax) {} //초기화

    pmp::Point pmin; // 최소 좌표
    pmp::Point pmax; // 최대 좌표

    bool Intersect(const AABB& other);          // 다른 AABB와의 교차 여부 판단
    bool Intersect(const pmp::Point& p);        // 특정 점과의 교차 여부 판단
    bool Intersect(const pmp::Point& p, const pmp::Point& d);  // 특정 점에서 방향 벡터를 따라 교차 여부 판단

    AABB Merge(const AABB& other);  // 다른 AABB와 합침
    AABB Merge(const pmp::Point &p);
    int LongestAxis();              // 가장 긴 축 반환
    pmp::Point Center();            // 중심점 반환

    void Split(AABB& left, AABB& right);  // 노드를 두 개로 분할
};

class BV {
public:
    AABB aabb;                        // AABB 객체
    std::vector<int> face_index;      // 면 인덱스 목록
    BV* left = nullptr;             // 왼쪽 자식 노드
    BV* right = nullptr;            // 오른쪽 자식 노드
    int level = 0;                    // 노드 레벨
    float color[3] = { 0.0f, 0.0f, 0.0f }; // 색상

    BV() = default;  // 기본 생성자
};