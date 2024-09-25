# OpenGL과 PMP 라이브러리를 이용한 BVH 및 BVTT 구현

이 프로젝트는 **PMP Surface Mesh 라이브러리**와 **OpenGL**을 사용하여 **Bounding Volume Hierarchy (BVH)**와 **Bounding Volume Test Tree (BVTT)**를 구현한 예제입니다. 이 코드는 3D 모델 간의 충돌 탐지를 효율적으로 수행하기 위한 구조를 포함합니다.

## 주요 기능

- **BVH (Bounding Volume Hierarchy)**: 3D 모델의 면들을 감싸는 AABB(축에 정렬된 경계 상자)를 기반으로 한 계층적 구조입니다. BVH는 충돌 감지 및 렌더링 시 효율성을 높이는 데 사용됩니다.
- **BVTT (Bounding Volume Test Tree)**: 두 BVH 구조 간의 충돌을 효율적으로 테스트하기 위해 사용되는 자료 구조입니다. BVTT는 두 개의 BVH 트리의 각 노드를 비교하여 충돌 여부를 판단합니다.
- **AABB (Axis-Aligned Bounding Box)**: 각 BV가 포함하는 삼각형들의 경계 상자를 계산하여 사용합니다. AABB는 교차 여부를 판단하기 위한 기본 단위입니다.

## 구현 내용

- **BVH 구축**: `BVH::Build()` 메서드는 모델의 모든 면을 재귀적으로 AABB로 묶어 BVH 트리를 구성합니다. 각 노드에 대해 AABB를 계산하고, 자식 노드로 재귀적으로 분할합니다.
- **BVTT 구축 및 충돌 검사**: `BVTT::Build()`는 두 BVH 트리 간의 충돌 가능성을 탐지하고 충돌한 AABB를 화면에 시각적으로 표시합니다. 충돌 처리 및 리스트 관리를 통해 충돌된 영역을 효율적으로 찾습니다.
- **시각화**: OpenGL을 이용해 BVH와 BVTT의 구조를 화면에 그리며, 충돌된 AABB는 붉은색으로 표시됩니다.

## 파일 설명

- `BV.h`, `BV.cpp`: Bounding Volume에 대한 정의 및 AABB 교차 판정, 분할 등의 기능이 포함되어 있습니다.
- `BVH.h`, `BVH.cpp`: 3D 모델을 위한 BVH 트리 구축 및 검증 기능을 포함합니다.
- `BVTT.h`, `BVTT.cpp`: 두 BVH 간의 충돌을 효율적으로 탐지하기 위한 BVTT 트리 구축 및 충돌 처리 로직이 구현되어 있습니다.
- `DrawComponent.h`, `DrawComponent.cpp`: OpenGL을 이용한 시각화 및 충돌 탐지 결과를 화면에 출력하는 코드입니다.

## 빌드 및 실행 방법

1. **필수 라이브러리 설치**:
    - [PMP Surface Mesh](https://pmp-library.github.io/)
    - OpenGL 및 GLUT
    ```bash
    sudo apt-get install freeglut3-dev
    ```

2. **프로젝트 빌드**:
    - Qt 환경에서 프로젝트를 설정하고, 필요한 라이브러리와 함께 컴파일합니다.
    - `qmake` 또는 `cmake`를 사용하여 프로젝트를 빌드합니다.

3. **실행**:
    - 실행 시 두 3D 모델의 BVH를 생성하고, 충돌된 영역을 확인할 수 있습니다.
    - 터미널에 충돌 탐지 방식(BVTT 또는 BF)별로 걸린 시간이 출력됩니다.

## 참고 사항

- 현재 예제는 두 개의 `icosahedron.obj` 모델을 사용하며, 하나의 모델을 x축으로 약간 이동시켜 충돌 여부를 시각화합니다.
- 충돌 검사는 **BVTT 방식**과 **BF 방식(Brute Force)**으로 나뉘어 각각의 성능을 비교할 수 있습니다.

## 라이선스

이 프로젝트는 MIT 라이선스를 따릅니다.
