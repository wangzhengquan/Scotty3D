
#include "halfedge.h"
#include "indexed.h"

#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it cannot perform an operation (i.e., because
    the resulting mesh does not have a valid representation).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/


/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius) {
	//faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3) return std::nullopt;


	std::vector< VertexRef > face_vertices;
	//In order to make the first edge point in the +x direction, first vertex should
	// be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s) {
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	//assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false); //the face to return
	FaceRef boundary = emplace_face(true); //the boundary loop around the face

	std::vector< HalfedgeRef > face_halfedges; //will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s) {
		//will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s+1)%sides];

		//h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		//t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		//e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); //false: non-sharp

		//set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f,-1.0f);

		//thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0) face->halfedge = h;
		if (s + 1 == sides) boundary->halfedge = t;

		//halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s) {
		face_halfedges[s]->next = face_halfedges[(s+1)%sides];
		face_halfedges[(s+1)%sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}


/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e) {
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); //set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; //copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); //set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); //set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
	FaceRef f_not_used = emplace_face();
	HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); //unchanged

	//n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;
	
	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e); // unchanged
	//h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	//t->face unchanged


	// Phase 4: Delete unused elements
	erase_face(f_not_used);
	erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}


/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e) {
	// A2L2 (REQUIRED): split_edge
	// Phase 1: collect existing elements
	VertexRef vm = *bisect_edge(e);
	HalfedgeRef h1 = vm->halfedge;
	HalfedgeRef t1 = h1->twin;
	HalfedgeRef t2 = t1->next;
	HalfedgeRef h2 = t2->twin;

	HalfedgeRef h3 = t2->next;
	HalfedgeRef h4 = h3->next;

	HalfedgeRef h5 = h1->next;
	HalfedgeRef h6 = h5->next;

	FaceRef f1 = t1->face;
	FaceRef f2 = h2->face;
	 
	if(!f1->boundary) {
		FaceRef fnew = emplace_face();
		EdgeRef enew = emplace_edge();
		HalfedgeRef hnew = emplace_halfedge();
		HalfedgeRef tnew = emplace_halfedge();

		fnew->halfedge = tnew;
		enew->halfedge = hnew;

		hnew->twin = tnew;
		hnew->next = h4;
		hnew->vertex = vm;
		hnew->edge = enew;
		hnew->face = f1;

		tnew->twin = hnew;
		tnew->next = t2;
		tnew->vertex = h4->vertex;
		tnew->edge = enew;
		tnew->face = fnew;
		 
		t1->next = hnew;
		t2->face = fnew;
		h3->face = fnew;
		h3->next = tnew;
		

		f1->halfedge = hnew;
	}

	if(!f2->boundary) {
		FaceRef fnew = emplace_face();
		EdgeRef enew = emplace_edge();
		HalfedgeRef hnew = emplace_halfedge();
		HalfedgeRef tnew = emplace_halfedge();

		fnew->halfedge = tnew;
		enew->halfedge = hnew;

		hnew->twin = tnew;
		hnew->next = h6;
		hnew->vertex = vm;
		hnew->edge = enew;
		hnew->face = f2;

		tnew->twin = hnew;
		tnew->next = h1;
		tnew->vertex = h6->vertex;
		tnew->edge = enew;
		tnew->face = fnew;
		 
		h2->next = hnew;
		h1->face = fnew;
		h5->face = fnew;
		h5->next = tnew;

		f2->halfedge = hnew;
	}
	return vm;
}



/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f) {
	// A2Lx4 (OPTIONAL): inset vertex
	if (f->boundary) return std::nullopt;
	std::vector<HalfedgeRef> halfedges;
	std::vector<VertexRef> vertices;
	std::vector<VertexCRef> vertices_c;
	std::vector<HalfedgeRef> new_halfedges;
	std::vector<FaceRef> new_faces;
	HalfedgeRef h = f->halfedge;
	do {
		halfedges.push_back(h);
		vertices.push_back(h->vertex);
		vertices_c.push_back(h->vertex);
		new_halfedges.push_back(emplace_fulledge());
		new_faces.push_back(emplace_face());
		h = h->next;
	} while(h !=f->halfedge);

	VertexRef vm = emplace_vertex();
	vm->position = f->center();
	interpolate_data(vertices_c, vm);

	size_t n = halfedges.size();
	for (size_t i = 0; i < n; i++) {
		if(i == 0) {
			vm->halfedge = new_halfedges[i]->twin;
		}
		new_halfedges[i]->set_nvf(new_halfedges[(i + n - 1) % n]->twin, vertices[i], new_faces[(i + n - 1) % n]);
		new_halfedges[i]->twin->set_nvf(halfedges[i], vm, new_faces[i]);
		halfedges[i]->set_nf(new_halfedges[(i + 1) % n], new_faces[i]);
		new_faces[i]->halfedge = new_halfedges[i]->twin;
	}
	erase_face(f);
	return vm;
}


/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v) {
	//A2Lx5 (OPTIONAL): Bevel Vertex
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in bevel_vertex_helper (A2Lx5h)

	(void)v;
    return std::nullopt;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e) {
	//A2Lx6 (OPTIONAL): Bevel Edge
	// Reminder: This function does not update the vertex positions.
	// remember to also fill in bevel_edge_helper (A2Lx6h)

	(void)e;
    return std::nullopt;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f) {
	//A2L4: Extrude Face
	// Reminder: This function does not update the vertex positions.
	// Remember to also fill in Extrude Positions Helper (A2L4h)

	// Collect the necessary halfedges and vertices
	std::vector<HalfedgeRef> face_halfedges;
	std::vector<VertexRef> face_vertices;
  auto copyVertex = [&](VertexRef v) -> VertexRef {
		VertexRef new_v = emplace_vertex();
		new_v->position = v->position;
		interpolate_data({v}, new_v); // Copy data from the original vertex
		return new_v;
	};

 
	HalfedgeRef h = f->halfedge;
	do {
		face_halfedges.push_back(h);
		face_vertices.push_back(h->vertex);
		h = h->next;
	} while (h != f->halfedge);

	size_t n = face_halfedges.size();
	// FaceRef new_f = emplace_face();
	std::vector<HalfedgeRef> new_halfedges, new_corner_halfedges;
	
	std::vector<FaceRef> new_faces;
	std::vector<VertexRef> new_vertices;
	
	// Create new halfedges and vertices
	for (size_t i = 0; i < n; ++i) {
		new_halfedges.emplace_back(emplace_fulledge());
		new_corner_halfedges.emplace_back(emplace_fulledge());

		new_vertices.emplace_back(copyVertex(face_halfedges[i]->vertex));
		new_faces.emplace_back(emplace_face());
	}

	for (size_t i = 0; i < n; ++i) {
		new_halfedges[i]->set_nvf(new_halfedges[(i + 1) % n], new_vertices[i], f);
		new_halfedges[i]->twin->set_nvf( new_corner_halfedges[i], new_vertices[(i + 1) % n],  new_faces[i]);

		face_halfedges[i]->set_nf(new_corner_halfedges[(i+1)%n]->twin, new_faces[i]);

		new_corner_halfedges[i]->set_nvf( face_halfedges[i], new_vertices[i],  new_faces[i]);
		new_corner_halfedges[i]->twin->set_nvf( new_halfedges[(i + n - 1) % n]->twin, face_vertices[i],  new_faces[(i + n - 1) % n]);

		new_vertices[i]->halfedge = new_halfedges[i];
		new_faces[i]->halfedge = face_halfedges[i];
	}

	f->halfedge = new_halfedges.front();
	return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e) {
	//A2L1: Flip Edge
	// Check if the edge is a boundary edge
	if (e->on_boundary()) {
			return std::nullopt;
	}

	// Collect the necessary halfedges
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;

	HalfedgeRef h_next = h->next;
	HalfedgeRef h_prev = *prev(h);
	// std::cout << "h_prev:" << std::to_string(h_prev->id)<< std::endl;
	HalfedgeRef h_next_next = h_next->next;

	HalfedgeRef t_next = t->next;
	HalfedgeRef t_prev = *prev(t);
	// std::cout << "t_prev:" << std::to_string(t_prev->id)<<  std::endl;
	HalfedgeRef t_next_next = t_next->next;

	// Collect the necessary vertices
	VertexRef hv = h->vertex;          // Vertex at the start of h1
	VertexRef tv = t->vertex;          // Vertex at the start of h2
	VertexRef hnnv = h_next_next->vertex;     // Vertex at the end of h1_prev
	VertexRef tnnv = t_next_next->vertex;     // Vertex at the end of h2_prev

	// Collect the necessary faces
	FaceRef fh = h->face;
	FaceRef ft = t->face;

	// Reassign the connectivity
	h->next = h_next_next;
	h_prev->next = t_next;
	t_next->next = h;
	 
	t->next = t_next_next;
	t_prev->next = h_next;
	h_next->next = t;

	// Update the vertex pointers
	h->vertex = tnnv;  // h now points to v4
	t->vertex = hnnv;  // t now points to v3

	// Update the face pointers
	h_next->face = ft;
	t_next->face = fh;
	ft->halfedge = h_next;
	fh->halfedge = t_next;


	// Update the halfedge pointers for the vertices
	hv->halfedge = t_next;
	tv->halfedge = h_next;
	hnnv->halfedge = t;
	tnnv->halfedge = h;

	// Return the edge that was flipped
	return e;
}


/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face) {
	//A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; //TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v) {
	// A2Lx1 (OPTIONAL): Dissolve Vertex
	std::vector<HalfedgeRef> merged_face_halfedges, erased_halfedges;
	Halfedge_Mesh::HalfedgeRef he = v->halfedge;
	Halfedge_Mesh::HalfedgeRef heOrig = he, pre;

	do {
		Halfedge_Mesh::HalfedgeRef tempOrig = he;
		erased_halfedges.push_back(tempOrig);
		
		for(pre = he, he = he->next; he->next != tempOrig; pre = he, he = he->next) {
			if(!tempOrig->face->boundary) {
				merged_face_halfedges.push_back(he);
			}
		}
		 
		if(tempOrig->face->boundary) {
			HalfedgeRef newh = emplace_fulledge();
			HalfedgeRef newt= newh->twin;
			newh->vertex = tempOrig->twin->vertex;
			newt->set_nvf(tempOrig->next, he->vertex, tempOrig->face);
			pre->next = newt;
			newt->face->halfedge = newt;
			
			merged_face_halfedges.push_back(newh);
		}
		he = he->twin;
	} while (he != heOrig);

	if(erased_halfedges.size() < 3) {
		return std::nullopt;
	}

	size_t n = merged_face_halfedges.size();
	
	FaceRef f = emplace_face();
	f->halfedge = merged_face_halfedges[0];
	for(size_t i = 0; i < n; i++) {
		merged_face_halfedges[i]->vertex->halfedge = merged_face_halfedges[i];
		merged_face_halfedges[i]->twin->vertex->halfedge = merged_face_halfedges[i]->twin;
		merged_face_halfedges[i]->set_nf(merged_face_halfedges[(i + 1) % n], f);
	}

	for(HalfedgeRef he : erased_halfedges) {
		if(!he->face->boundary) {
			erase_face(he->face);
		}
		erase_fulledge(he);
	}
	erase_vertex(v);

	return f;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e) {
	// A2Lx2 (OPTIONAL): dissolve_edge
	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data
	if(this->faces.size() < 3) {
		return std::nullopt;
	}
	std::vector<HalfedgeRef> halfedges1, halfedges2;
	HalfedgeRef h = e->halfedge;
	h = h->face->boundary ? h : h->twin ;
	FaceRef f = h->face;

	HalfedgeRef h1 = h;
	while((h1 = h1->next) != h) {
		halfedges1.push_back(h1);
	}

	HalfedgeRef h2 = h->twin;
	while((h2 = h2->next) != h->twin) {
		halfedges2.push_back(h2);
		h2->face=f;
	}
	f->halfedge = halfedges1.front();
	h->vertex->halfedge = h->twin->next;
	h->twin->vertex->halfedge = h->next;
	halfedges1.back()->next = halfedges2.front();
	halfedges2.back()->next = halfedges1.front();
	 
	erase_face(h->twin->face);
	erase_fulledge(h);
  return f;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e) {
	//A2L3: Collapse Edge

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
	std::unordered_map< std::pair< Index, Index >, HalfedgeRef > halfedgesMap; //for quick lookup of halfedges by from/to vertex index
	std::set<FaceRef> facesToErase;
	std::set<HalfedgeRef> halfedgesToErase;
	std::set<EdgeRef> edgesToErase;
	// Collect the necessary halfedges
	HalfedgeRef h1 = e->halfedge;
	HalfedgeRef h2 = h1->twin;

	// Collect the necessary vertices
	VertexRef v1 = h1->vertex;
	VertexRef v2 = h2->vertex;

	// Collect the necessary faces
	FaceRef f1 = h1->face;
	FaceRef f2 = h2->face;

	// Create a new vertex at the midpoint of the edge
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); // Interpolate bone weights

	halfedgesToErase.emplace(h1);
	halfedgesToErase.emplace(h2);
	edgesToErase.emplace(e);

	f1->halfedge = h1->next;
	f2->halfedge = h2->next;

	// Collect the necessary halfedges around v1 and v2
	std::vector<HalfedgeRef> v1_halfedges;
	std::vector<HalfedgeRef> v2_halfedges;

	HalfedgeRef h = v1->halfedge;
	bool v1_hasBoundryHalfedge = false, v2_hasBoundryHalfedge = false;
	do {
			v1_halfedges.push_back(h);
			v1_hasBoundryHalfedge = v1_hasBoundryHalfedge || h->face->boundary;
			h = h->twin->next;
	} while (h != v1->halfedge);

	h = v2->halfedge;
	do {
			v2_halfedges.push_back(h);
			v2_hasBoundryHalfedge = v2_hasBoundryHalfedge || h->face->boundary;
			h = h->twin->next;
	} while (h != v2->halfedge);

	if (v1_hasBoundryHalfedge && v2_hasBoundryHalfedge && !e->on_boundary()) {
		return std::nullopt;
	}
	 
	// Update the vertex pointers for the surrounding halfedges
	for (HalfedgeRef he : v1_halfedges) {
		assert(he->vertex == v1);
		he->vertex = vm;
		auto line = std::make_pair(he->vertex->id, he->twin->vertex->id);
		halfedgesMap.emplace(line, he);
		if (he->twin->next == h1) {
			he->twin->next = he->twin->next->next;
		}
	}

	for (HalfedgeRef he : v2_halfedges) {
		assert(he->vertex == v2);
		he->vertex = vm;
		auto line = std::make_pair(he->vertex->id, he->twin->vertex->id);
		auto colineHe = halfedgesMap.find(line);
		if(colineHe != halfedgesMap.end()) {
			if(colineHe->second->face == f2) {
				facesToErase.emplace(f2);
				halfedgesToErase.emplace(colineHe->second);
				halfedgesToErase.emplace(he->twin);
				edgesToErase.emplace(colineHe->second->edge);

				he->twin->vertex->halfedge = colineHe->second->twin;

				he->edge->halfedge = he;
				he->twin = colineHe->second->twin;
				he->twin->edge = he->edge;
				he->twin->twin = he;
				
				halfedgesMap.insert_or_assign(line, he);
			} else if (he->face == f1) {
				facesToErase.emplace(f1);
				halfedgesToErase.emplace(colineHe->second->twin);
				halfedgesToErase.emplace(he);
				edgesToErase.emplace(he->edge);
				colineHe->second->twin->vertex->halfedge = he->twin;
				
				colineHe->second->edge->halfedge = colineHe->second;
				colineHe->second->twin = he->twin;
				colineHe->second->twin->edge = colineHe->second->edge;
				colineHe->second->twin->twin = colineHe->second;
				// halfedgesMap.emplace(line, he);
				
			}
		} else {
			if (he->twin->next == h2) {
				he->twin->next = he->twin->next->next;
			}
		}
	}

	for (HalfedgeRef he : v1_halfedges){
		// std::cout << he->id<< ":" << halfedgesToErase.count(he) <<std::endl;
		if (halfedgesToErase.count(he) == 0) {
			vm->halfedge = he;
			break;
		}
	}
	 
	// Remove the collapsed elements
	for (FaceRef f: facesToErase){
		erase_face(f);
	}
	for (auto h: halfedgesToErase){
		erase_halfedge(h);
	}
	for (auto e: edgesToErase){
		erase_edge(e);
	}
	
	erase_vertex(v1);
	erase_vertex(v2);

	// Return the newly created vertex
	return vm;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f) {
	//A2Lx3 (OPTIONAL): Collapse Face

	//Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	// (also works for bone_weights data on vertices!)
	if(f->boundary) {
		return std::nullopt;
	}
	std::vector<HalfedgeRef> halfedges;
	std::vector<VertexRef> vertices;
	std::vector<VertexCRef> vertices_c;
	HalfedgeRef h = f->halfedge;
	do {
		if(h->twin->face->boundary) {
			return std::nullopt;
		}
		halfedges.push_back(h);
		vertices.push_back(h->vertex);
		vertices_c.push_back(h->vertex);
		// corner_halfedges.push_back(h->twin->next);
		h = h->next;
	} while(h !=f->halfedge);

	VertexRef vm = emplace_vertex();
	vm->position = f->center();
	interpolate_data(vertices_c, vm);
	size_t n = halfedges.size();
	for(size_t i = 0; i < n; i++ ) {
		// set vertex of halfedges which originally refrenced to v to vm .
		VertexCRef v = vertices[i];
		HalfedgeRef h = v->halfedge;
		do {
			h->vertex = vm;
			h = h->twin->next;
		} while (h != v->halfedge);
	}

	for(size_t i = 0; i < n; i++ ) {
		// connect prev edge of h to next edge of h;
		HalfedgeRef h = halfedges[i]->twin;
		HalfedgeRef head = h->next;
		HalfedgeRef tail = head;
		while (tail->next != h) {
			tail = tail->next;
		}
		tail->next = head;
		head->face->halfedge = head;
		if(i == 0) {
			vm->halfedge = head;
		}
	}

	for(auto h : halfedges){
		erase_fulledge(h);
	}
	for(VertexRef v: vertices){
		erase_vertex(v);
	}
	erase_face(f);
	return vm;
  // return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2) {
	//A2Lx8: Weld Edges

	//Reminder: use interpolate_data() to merge bone_weights data on vertices!

    return std::nullopt;
}



/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance) {
	//A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper
	
	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.
	
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 * 
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink) {
	//A2L4h: Extrude Positions Helper

	//General strategy:
	// use mesh navigation to get starting positions from the surrounding faces,
	// compute the centroid from these positions + use to shrink, offset by move

	// Compute the centroid of the face
	// Vec3 centroid = face->center();
	// Vec3 normal = face->normal();

	HalfedgeRef h = face->halfedge;
	do {
			h->vertex->position += move; // Move along the normal
			h->vertex->position *= (1.0f - shrink); // Shrink toward the centroid
			h = h->next;
	} while (h != face->halfedge);

	 
	
}

