#include "test.h"
#include "geometry/halfedge.h"
#include "geometry/util.h"
#include <iostream>

static void expect_collapse(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &expect) {

	if (auto ret = mesh.collapse_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, expect, Test::CheckAllBits)) {
			std::cout << "\nexpect:\n" << expect.describe() << std::endl;
			throw Test::error("Resulting mesh did not match expected: " + *difference);

		
		}
	} else {
		throw Test::error("collapse_edge rejected operation!");
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
2--3--4
|  | /
5--6/

Collapse Edge on Edge: 2-3

After mesh:
0-----1\
 \   /  \
  \ /    \
   2------3
  / \    /
 /   \  /
4-----5/
*/
Test test_a2_l3_collapse_edge_basic_simple("a2.l3.collapse_edge.basic.simple", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 Vec3(-1.2f, 0.0f, 0.0f),   	 Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 3, 1}, 
		{2, 5, 6, 3}, 
		{1, 3, 4}, 
		{3, 6, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 			Vec3(0.0f, 0.0f, 0.0f),  			Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{2, 4, 5}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_collapse(mesh, edge, after);
});

/*
   0        
  / \   
 /   \       
1-----2 
*/
Test test_a2_l3_collapse_edge_single_triangle("a2.l3.collapse_edge.single.triangle", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
								Vec3(0.0f, 2.0f, 0.0f),
		Vec3(-1.0f, 1.0f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		 
	}, {
		{0, 1, 2},
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;
 	auto ret = mesh.collapse_edge(edge);
	assert(!ret);
	 
});

/**           
   / 0 \   
  /     \       
  1-----2 
	\     /
	 \ 3 / 
Collapse Edge on Edge: 1-2
*/
Test test_a2_l3_collapse_edge_double_triangle("a2.l3.collapse_edge.double.triangle", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
								Vec3(0.0f, 2.0f, 0.0f),
		Vec3(-1.0f, 1.0f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
								Vec3(0.0f, -1.0f, 0.0f),
		 
	}, {
		{0, 1, 2},
		{1, 3, 2},
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;
 	auto ret = mesh.collapse_edge(edge);
	assert(!ret);
	 
});

/**
   0 \---/ 1                                        / 0
	 |  \ /  |                                       /  |
	 |   2   |   (Collapse Edge on Edge: 0-2)       1   | 
	 |  / \  |                                     / \  |
	 3 /---\ 4                                  2 /---\ 3
 */
Test test_a2_l3_collapse_edge_four_triangle("a2.l3.collapse_edge.four.triangle", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f),  Vec3(1.1f, 1.0f, 0.0f),
								 Vec3(0.6f, 0.0f, 0.0f),
		Vec3(-1.2f, -1.0f, 0.0f),  Vec3(2.3f, -1.0f, 0.0f),
	}, {
		{0, 3, 2}, 
		{0, 2, 1},
		{1, 2, 4}, 
		{2, 3, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;
   
	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		 						 Vec3(1.1f, 1.0f, 0.0f),
							edge->center(),
		Vec3(-1.2f, -1.0f, 0.0f),  Vec3(2.3f, -1.0f, 0.0f),
	}, {
		{0, 1, 3}, 
		{1, 2, 3}
	});

	expect_collapse(mesh, edge, after);
});

/*
	 
  / 0 \ 		   / 0 \ 
 /     \ 			/     \ 						 
1---2---3    1 \   / 2
|   |   |    |  \ /  |
|   |   | -> |   3   | (Collapse Edge on Edge: 2-5)
|   |   |    |  / \  |
4---5---6    4 /   \ 5
 \     /      \     /
  \ 7 /        \ 6 /
  
*/
Test test_a2_l3_collapse_edge_basic_hard("a2.l3.collapse_edge.basic.hard", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
														  Vec3(0.0f, 2.0f, 0.0f),
		Vec3(-1.0f, 1.0f, 0.0f),  Vec3(0.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.2f, -1.0f, 0.0f), Vec3(1.2f, -1.0f, 0.0f),  Vec3(2.3f, -1.0f, 0.0f),
		 													Vec3(1.2f, -2.0f, 0.0f), 
	}, {
		{1, 4, 5, 2},
		{0, 1, 2, 3}, 
		{2, 5, 6, 3},
		{4, 7, 6, 5}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
							  Vec3(0.0f, 2.0f, 0.0f),
		Vec3(-1.0f, 1.0f, 0.0f),  Vec3(1.1f, 1.0f, 0.0f),
								Vec3(0.6f, 0.0f, 0.0f),
		Vec3(-1.2f, -1.0f, 0.0f),  Vec3(2.3f, -1.0f, 0.0f),
								Vec3(1.2f, -2.0f, 0.0f), 
	}, {
		{0, 1, 3, 2}, 
		{1, 4, 3},
		{2, 3, 5},
		{3, 4, 6, 5}
	});

	expect_collapse(mesh, edge, after);
});



/*
0---1---2    0 \   / 1
|   |   |    |  \ /  |
|   |   | -> |   2   | 
|   |   |    |  / \  |
3---4---5    3 /   \ 4
*/
Test test_a2_l3_collapse_edge_basic_hard2("a2.l3.collapse_edge.basic.hard2", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		Vec3(-1.2f, -1.0f, 0.0f), Vec3(1.2f, -1.0f, 0.0f),  Vec3(2.3f, -1.0f, 0.0f)
		 
	}, {
		{0, 3, 4, 1}, 
		{1, 4, 5, 2}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;
  auto ret = mesh.collapse_edge(edge);
	assert(!ret);
	 
});

/*
EDGE CASE

Initial mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

Collapse Edge on Edge: 0-1

After mesh:
    0--\
   / \  \
  /   \  \
 /     1--2
/      | /
3------4/
*/
Test test_a2_l3_collapse_edge_edge_boundary("a2.l3.collapse_edge.edge.boundary", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{0, 4, 5, 2}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		       Vec3(0.05f, 1.05f, 0.0f), 
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 1, 2}, 
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});

	expect_collapse(mesh, edge, after);
});

/*
EDGE CASE

Initial mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

Collapse Edge on Edge: 2-3

After mesh:
0--1
|\ |
| \2
|  | 
3--4
*/
Test test_a2_l3_collapse_edge_edge_boundary2("a2.l3.collapse_edge.edge.boundary2", []() {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{1, 2, 3}, 
		{0, 2, 1}, 
		{0, 4, 5, 2}, 
		{2, 5, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                             Vec3(1.75f, 0.0f, 0.0f), 
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{0, 3, 4, 2}
	});

	expect_collapse(mesh, edge, after);
});

Test test_a2_l3_collapse_edge_complex1("a2.l3.collapse_edge.complex1", []() {

	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
							Vec3(-0.45f, 0.825f, 0.0f), 
													Vec3(1.15f, 0.5f, 0.0f),
		 Vec3(-0.75f, -0.525f, 0.0f), 						
		 Vec3(-1.0f, -1.0f, 0.0f),  Vec3(1.0f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{2, 4, 1}, 
		{2, 3, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;
	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
																edge->center(),
		Vec3(-0.75f, -0.525f, 0.0f), 						
		Vec3(-1.0f, -1.0f, 0.0f),  Vec3(1.0f, -1.0f, 0.0f)
	}, {
		{0, 1, 3}, 
		{1, 2, 3}
	});

	expect_collapse(mesh, edge, after);
});

 
Test test_a2_l3_collapse_tetrahedron("a2.l3.collapse_edge.triangular.pyramid", []() {
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

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		 Vec3{0.0f, 1.0f, 0.0f},  
		 edge->center(), 						
		 Vec3{1.0f * std::cos(2.f * 2.f * PI_F / 3), 2.0f, 1.0f * std::sin(0.f * 2.f * PI_F / 3)}, 
	}, {
		{0, 2, 1}, 
		{0, 1, 2}
	});

	expect_collapse(mesh, edge, after);
});

Test test_a2_l3_collapse_edge_custom1("a2.l3.collapse_edge.custom1", []() {
	Util::Gen::Data data = Util::Gen::custom1();
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_mesh(Indexed_Mesh(std::move(data.verts), std::move(data.elems)));
	Halfedge_Mesh::EdgeRef edge = mesh.edges.end();
	for (auto e = mesh.edges.begin(); e != mesh.edges.end(); ++e) {
		if(e->halfedge->vertex->id == 3 && e->halfedge->twin->vertex->id == 4) {
			edge = e;
			break;
		}
	}
	assert(edge != mesh.edges.end());
	if (auto ret = mesh.collapse_edge(edge)) {
		if (auto msg = mesh.validate()) {
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
	}
});

