# GPU 3D Algorithm

AABB 기반 BVH/BVTT를 직접 구현하고 OpenGL 시각화로 3D 충돌 감지 성능을 검증한 C++ 그래픽스 알고리즘 프로젝트입니다.

> Portfolio position: graphics algorithm implementation, spatial data structures, measurable performance improvement.

## Problem

3D 모델 간 충돌 감지를 모든 삼각형 쌍에 대해 brute force로 계산하면 모델 크기가 커질수록 비용이 급격히 증가합니다. 이 프로젝트는 Bounding Volume Hierarchy(BVH)와 Bounding Volume Test Tree(BVTT)를 직접 구현해 충돌 후보를 줄이고, 결과를 OpenGL로 시각화했습니다.

## What I Built

- AABB(Axis-Aligned Bounding Box) 자료구조
- BVH tree construction for triangle meshes
- BVTT traversal for pairwise collision detection
- Brute Force / BVH / BVTT performance comparison
- OpenGL debug rendering for collision regions
- Benchmark and profiling flow for runtime comparison

## My Role

개인 프로젝트로 알고리즘 설계, C++ 구현, OpenGL 시각화, 성능 측정까지 단독 수행했습니다. 기존 OBJ 렌더링 기반 코드를 분석하고, 충돌 감지 구조와 성능 비교 기능을 확장했습니다.

## Stack

| Area | Stack |
| --- | --- |
| Language | C++ |
| Graphics | OpenGL, GLUT |
| Geometry | PMP, triangle mesh processing |
| Algorithm | AABB, BVH, BVTT, spatial partitioning |
| Build | CMake / Qt environment |

## Run

```bash
cmake -S . -B build
cmake --build build
```

Qt/OpenGL 환경이 필요한 경우, 프로젝트를 Qt Creator에서 열고 OpenGL/GLUT/PMP 의존성을 맞춘 뒤 실행합니다.

## Validation Evidence

Test data: `icosahedron.obj`, 12,182 triangles, two-model collision detection.

| Method | Complexity | Measured time | Memory | Accuracy |
| --- | --- | ---: | ---: | ---: |
| Brute Force | O(n^2) | 847ms | 45MB | 100% |
| BVH | O(n log n) | 342ms | 52MB | 100% |
| BVTT | O(log n) traversal path | 126ms | 38MB | 100% |

Key result: BVTT reduced measured processing time from 847ms to 126ms, about 85% faster than brute force on the tested scene.

## Demo / Screenshot

![Collision visualization](https://github.com/user-attachments/assets/15d404f8-4dba-441c-a525-c1ce48ca10a3)

## Why It Matters

이 프로젝트는 그래픽스/엔진 개발에서 중요한 공간 자료구조, 충돌 감지, 디버그 시각화, 성능 검증을 하나의 작은 시스템으로 구현한 사례입니다. 단순히 알고리즘을 설명하는 것이 아니라, 측정 가능한 처리 시간과 정확도 비교를 남긴 것이 핵심입니다.
