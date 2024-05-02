#include "DrawComponent.h"
#include "gl/freeglut.h"

#include"pmp/io/io.h"
#include"pmp/algorithms/normals.h"

using namespace std;

void DrawComponent::Init() {
    pmp::read(mesh, "obj\\bunny_stanford.obj");
    pmp::vertex_normals(mesh);
    cout << "Loaded mesh with " << mesh.n_vertices() << " vertices and " << mesh.n_faces() << " faces." << endl;


    bvh = new BVH(mesh);
    bvh->build();

    // BVH 구조의 검증 시작
    //bvh->validate();
}


void DrawComponent::Draw() {

    if (bvh) {

        bvh->draw(arrowNum0, arrowNum1);
        cout << arrowNum0 << endl;
    }
}