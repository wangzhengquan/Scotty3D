#include "test.h"
#include "geometry/halfedge.h"
#include <iostream>

static void expect_flip(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &expect) {
	// std::cout << "\nbefore:\n" << mesh.describe() << std::endl;
	// if (auto msg = mesh.validate()) {
	// 	throw Test::error("Invalid mesh1: " + msg.value().second);
	// }
	if (auto ret = mesh.flip_edge(edge)) {
		// std::cout << "\nafter:\n" << mesh.describe() << std::endl;
		
		// std::cout << "\nexpect:\n" << expect.describe() << std::endl;
		
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check if returned edge is the same edge
		if (ret != edge) {
			throw Test::error("Did not return the same edge!");
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, expect, Test::CheckAllBits)) {
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	} else {
		throw Test::error("flip_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 1-4

After mesh:
0--1\
|\   \
| \---2
|    /
3--4/
*/
Test test_a2_l1_flip_edge_basic_simple("a2.l1.flip_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh expect = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 2}, 
		{0, 2, 1}
	});

	expect_flip(mesh, edge, expect);
});

Test test_a2_l1_flip_edge_basic_simple2("a2.l1.flip_edge.basic.simple2", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f), Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f) 
	}, {
		{0, 3, 2}, 
		{0, 2, 1}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh expect = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f), Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f)
	}, {
		{0, 3, 1}, 
		{1, 3, 2}
	});

	expect_flip(mesh, edge, expect);
});

/*
EDGE CASE

Initial mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/

Flip Edge on Edge: 3-4

After mesh:
0--1\
|  | \
|  |  2
|  | /
3--4/
*/
Test test_a2_l1_flip_edge_edge_boundary("a2.l1.flip_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                                            Vec3(2.2f, 0.0f, 0.0f),
		Vec3(-1.3f,-0.7f, 0.0f), Vec3(1.4f, -1.0f, 0.0f)
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});
	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	if (mesh.flip_edge(edge)) {
		throw Test::error("flip_edge should not work at the boundary.");
	}
});

Test test_a2_l1_flip_edge_tetrahedron("a2.l1.flip_edge.triangular.pyramid", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		 Vec3{0.0f, 1.0f, 0.0f},  
		 Vec3{1.0f * std::cos(0.f * 2.f * PI_F / 3), 0.0f, 1.0f * std::sin(0.f * 2.f * PI_F / 3)}, 						
		 Vec3{1.0f * std::cos(1.f * 2.f * PI_F / 3), 1.0f, 1.0f * std::sin(0.f * 2.f * PI_F / 3)}, 
		 Vec3{1.0f * std::cos(2.f * 2.f * PI_F / 3), 2.0f, 1.0f * std::sin(0.f * 2.f * PI_F / 3)}, 
	}, {
		{0, 2, 1}, 
		{0, 3, 2}, 
		{0, 1, 3},
		{1, 2, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	assert(!mesh.flip_edge(edge));
	 
});

