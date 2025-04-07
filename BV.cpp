#include "BV.h"


// �ٸ� AABB���� ���� ���� �Ǵ�
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

// Ư�� ������ ���� ���� �Ǵ�
bool AABB::Intersect(const pmp::Point& p)
{
    if (p[0] < pmin[0] || p[0] > pmax[0]) return false;
    if (p[1] < pmin[1] || p[1] > pmax[1]) return false;
    if (p[2] < pmin[2] || p[2] > pmax[2]) return false;
    return true;
}

// Ư�� ������ ���� ���͸� ���� ���� ���� �Ǵ�
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

// �ٸ� AABB�� ��ħ
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

// ���� �� �� ��ȯ
int AABB::LongestAxis()
{
    pmp::Point d = pmax - pmin;
    if (d[0] > d[1] && d[0] > d[2]) return 0;
    if (d[1] > d[2]) return 1;
    return 2;
}

// �߽��� ��ȯ
pmp::Point AABB::Center()
{
    return 0.5 * (pmin + pmax);
}

// ��带 �� ���� ����
void AABB::Split(AABB& left, AABB& right)
{
    int axis = LongestAxis();
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