#include "BV.h"


// 다른 AABB와의 교차 여부 판단
bool AABB::Intersect(const AABB& other)
{
    if (pmax[0] < other.pmin[0] || pmin[0] > other.pmax[0])
        return false;
    if (pmax[1] < other.pmin[1] || pmin[1] > other.pmax[1])
        return false;
    if (pmax[2] < other.pmin[2] || pmin[2] > other.pmax[2])
        return false;
    return true;
}

// 특정 점과의 교차 여부 판단
bool AABB::Intersect(const pmp::Point& p)
{
    if (p[0] < pmin[0] || p[0] > pmax[0]) return false;
    if (p[1] < pmin[1] || p[1] > pmax[1]) return false;
    if (p[2] < pmin[2] || p[2] > pmax[2]) return false;
    return true;
}

// 특정 점에서 방향 벡터를 따라 교차 여부 판단
bool AABB::Intersect(const pmp::Point& p, const pmp::Point& d)
{
    double tmin = 0.0;
    double tmax = 1.0;
    for (int i = 0; i < 3; i++) {
        if (d[i] == 0.0) {
            if (p[i] < pmin[i] || p[i] > pmax[i])
                return false;
        }
        else {
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

// 다른 AABB와 합침
AABB AABB::Merge(const AABB& other)
{
    pmp::Point newPmin, newPmax;
    for (int i = 0; i < 3; i++)
    {
        newPmin[i] = std::min(this->pmin[i], other.pmin[i]);
        newPmax[i] = std::max(this->pmax[i], other.pmax[i]);
    }
    return AABB(newPmin, newPmax);
}

// 가장 긴 축 반환
int AABB::LongestAxis()
{
    pmp::Point d = pmax - pmin;
    if (d[0] > d[1] && d[0] > d[2]) return 0;
    if (d[1] > d[2]) return 1;
    return 2;
}

// 중심점 반환
pmp::Point AABB::Center()
{
    return 0.5 * (pmin + pmax);
}

// 노드를 두 개로 분할
void AABB::Split(AABB& left, AABB& right)
{
    int axis = LongestAxis();
    double mid = (pmax[axis] + pmin[axis]) * 0.5;

    pmp::Point pmin_l = pmin;
    pmp::Point pmax_l = pmax;
    pmp::Point pmin_r = pmin;
    pmp::Point pmax_r = pmax;

    pmax_l[axis] = mid; // 왼쪽 상자의 최대 AABB 수정
    pmin_r[axis] = mid; // 오른쪽 상자의 최소 AABB 수정

    left = AABB(pmin_l, pmax_l);
    right = AABB(pmin_r, pmax_r);
}