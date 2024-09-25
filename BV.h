#pragma once
#include"pmp/surface_mesh.h"
#include <vector>

class AABB
{
public:
    AABB() // �⺻ ������
    {
        pmin = pmp::Point(std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        pmax = pmp::Point(std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());
    }
    AABB(pmp::Point pmin, pmp::Point pmax) : pmin(pmin), pmax(pmax) {} //�ʱ�ȭ

    pmp::Point pmin; // �ּ� ��ǥ
    pmp::Point pmax; // �ִ� ��ǥ

    bool Intersect(const AABB& other);          // �ٸ� AABB���� ���� ���� �Ǵ�
    bool Intersect(const pmp::Point& p);        // Ư�� ������ ���� ���� �Ǵ�
    bool Intersect(const pmp::Point& p, const pmp::Point& d);  // Ư�� ������ ���� ���͸� ���� ���� ���� �Ǵ�

    AABB Merge(const AABB& other);  // �ٸ� AABB�� ��ħ
    AABB Merge(const pmp::Point &p);
    int LongestAxis();              // ���� �� �� ��ȯ
    pmp::Point Center();            // �߽��� ��ȯ

    void Split(AABB& left, AABB& right);  // ��带 �� ���� ����
};

class BV {
public:
    AABB aabb;                        // AABB ��ü
    std::vector<int> face_index;      // �� �ε��� ���
    BV* left = nullptr;             // ���� �ڽ� ���
    BV* right = nullptr;            // ������ �ڽ� ���
    int level = 0;                    // ��� ����
    float color[3] = { 0.0f, 0.0f, 0.0f }; // ����

    BV() = default;  // �⺻ ������
};