#include "DrawComponent.h"
#include "gl/freeglut.h"

#include "pmp/io/io.h"
#include "pmp/algorithms/normals.h"

void DrawComponent::Init()
{
	//model load
	pmp::read(mesh, "models\\bear_bis.obj");

	pmp::vertex_normals(mesh);

	std::cout << "#f " << mesh.n_faces() << " #v " << mesh.n_vertices() << std::endl;
}

void DrawComponent::Draw()
{
	Node rootNode = createRoot(mesh);



	for (auto f : mesh.faces()) {
		Faces tempFace;
		int i = 0;
		for (auto v : mesh.vertices(f)) {
			auto p = mesh.position(v); // x y z

			tempFace.vertices[i].x = p[0];
			tempFace.vertices[i].y = p[1];
			tempFace.vertices[i].z = p[2];
			i++;

		}
		putNode()
	}

}
