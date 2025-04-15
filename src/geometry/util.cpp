#include "util.h"
#include "../scene/shape.h"
#include "../lib/stacktrace.h"
#include <map>
#include <set>
#include <iostream>


namespace Util {

Indexed_Mesh cyl_mesh(float radius, float height, uint32_t sides, bool cap) {
	return cone_mesh(radius, radius, height, sides, cap);
}

Indexed_Mesh arrow_mesh(float rbase, float rtip, float height) {
	Gen::Data base = Gen::cone(rbase, rbase, 0.75f * height, 10, true);
	Gen::Data tip = Gen::cone(rtip, 0.001f, 0.25f * height, 10, true);
	for (auto& v : tip.verts) v.pos.y += 0.7f;
	return Gen::merge(std::move(base), std::move(tip));
}

Indexed_Mesh scale_mesh() {
	Gen::Data base = Gen::cone(0.03f, 0.03f, 0.7f, 10, true);
	Gen::Data tip = Gen::cube(0.1f);
	for (auto& v : tip.verts) v.pos.y += 0.7f;
	return Gen::merge(std::move(base), std::move(tip));
}

Indexed_Mesh cone_mesh(float bradius, float tradius, float height, uint32_t sides, bool cap) {
	Gen::Data cone = Gen::cone(bradius, tradius, height, sides, cap);
	return Gen::dedup({std::move(cone.verts), std::move(cone.elems)});
}

Indexed_Mesh cyl_mesh_disjoint(float radius, float height, uint32_t sides) {
	Gen::Data cone = Gen::cone(radius, radius, height, sides, false);
	return Indexed_Mesh(std::move(cone.verts), std::move(cone.elems));
}

Indexed_Mesh torus_mesh(float iradius, float oradius, uint32_t segments, uint32_t sides) {
	Gen::Data torus = Gen::torus(iradius, oradius, segments, sides);
	return Gen::dedup({std::move(torus.verts), std::move(torus.elems)});
}

Indexed_Mesh cube_mesh(float r) {
	Gen::Data cube = Gen::cube(r);
	return Indexed_Mesh(std::move(cube.verts), std::move(cube.elems));
}

Indexed_Mesh square_mesh(float r) {
	Gen::Data square = Gen::quad(r, r);
	return Indexed_Mesh(std::move(square.verts), std::move(square.elems));
}

Indexed_Mesh quad_mesh(float x, float y) {
	Gen::Data square = Gen::quad(x, y);
	return Indexed_Mesh(std::move(square.verts), std::move(square.elems));
}

Indexed_Mesh pentagon_mesh(float r) {
	Gen::Data pentagon = Gen::pentagon(r);
	return Indexed_Mesh(std::move(pentagon.verts), std::move(pentagon.elems));
}

Indexed_Mesh texture_sphere_mesh(float r, uint32_t subdivisions) {
	Gen::Data ico_sphere = Gen::texture_ico_sphere(r, subdivisions);
	return Indexed_Mesh(std::move(ico_sphere.verts), std::move(ico_sphere.elems));
}

Indexed_Mesh closed_sphere_mesh(float r, uint32_t subdivisions) {
	Gen::Data ico_sphere = Gen::closed_ico_sphere(r, subdivisions);
	return Indexed_Mesh(std::move(ico_sphere.verts), std::move(ico_sphere.elems));
}

Indexed_Mesh hemi_mesh(float r) {
	Gen::Data hemi = Gen::uv_hemisphere(r);
	return Indexed_Mesh(std::move(hemi.verts), std::move(hemi.elems));
}

Indexed_Mesh capsule_mesh(float h, float r) {

	Gen::Data bottom = Gen::uv_hemisphere(r);
	Gen::Data top = Gen::uv_hemisphere(r);
	for (auto& v : top.verts) v.pos.y = -v.pos.y + h;
	Gen::Data cyl = Gen::cone(r, r, h, 64, false);

	Indexed_Mesh::Index cyl_off = (Indexed_Mesh::Index)bottom.verts.size();
	Indexed_Mesh::Index top_off = cyl_off + (Indexed_Mesh::Index)cyl.verts.size();

	for (auto& i : cyl.elems) i += cyl_off;
	for (auto& i : top.elems) i += top_off;

	bottom.verts.insert(bottom.verts.end(), cyl.verts.begin(), cyl.verts.end());
	bottom.elems.insert(bottom.elems.end(), cyl.elems.begin(), cyl.elems.end());

	bottom.verts.insert(bottom.verts.end(), top.verts.begin(), top.verts.end());
	bottom.elems.insert(bottom.elems.end(), top.elems.begin(), top.elems.end());

	return Indexed_Mesh(std::move(bottom.verts), std::move(bottom.elems));
}

GL::Lines spotlight_mesh(Spectrum color, float inner, float outer) {

	constexpr uint32_t steps = 72;
	constexpr float step = (2.0f * PI_F) / (steps + 1);
	constexpr float dist = 5.0f;

	inner = std::clamp(inner / 2.0f, 0.0f, 90.0f);
	outer = std::clamp(outer / 2.0f, 0.0f, 90.0f);
	float ri = dist * std::tan(Radians(inner));
	float ro = dist * std::tan(Radians(outer));
	Gen::LData iring = Gen::circle(color, ri, steps);
	Gen::LData oring = Gen::circle(color, ro, steps);
	Gen::LData rings = Gen::merge(std::move(iring), std::move(oring));
	for (auto& v : rings.verts) v.pos.y += 5.0f;

	float t = 0.0f;
	for (uint32_t i = 0; i < steps; i += steps / 4) {
		Vec3 point = ro * Vec3(std::sin(t), 0.0f, std::cos(t));
		rings.verts.push_back({{}, color});
		rings.verts.push_back({Vec3(point.x, 5.0f, point.z), color});
		t += step * (steps / 4);
	}
	return GL::Lines(std::move(rings.verts), 1.0f);
}

namespace Gen {

Indexed_Mesh dedup(Data&& d) {

	std::vector<Indexed_Mesh::Vert> verts;
	std::vector<Indexed_Mesh::Index> elems;

	// normals be damned
	std::map<Vec3, Indexed_Mesh::Index> v_to_idx;

	for (size_t i = 0; i < d.elems.size(); i++) {
		Indexed_Mesh::Index idx = d.elems[i];
		Indexed_Mesh::Vert v = d.verts[idx];
		auto entry = v_to_idx.find(v.pos);
		Indexed_Mesh::Index new_idx;
		if (entry == v_to_idx.end()) {
			new_idx = (Indexed_Mesh::Index)verts.size();
			v_to_idx.insert({v.pos, new_idx});
			verts.push_back(v);
		} else {
			new_idx = entry->second;
		}
		elems.push_back(new_idx);
	}

	return Indexed_Mesh(std::move(verts), std::move(elems));
}

Indexed_Mesh merge(Data&& l, Data&& r) {
	for (auto& i : r.elems) i += (Indexed_Mesh::Index)l.verts.size();
	l.verts.insert(l.verts.end(), r.verts.begin(), r.verts.end());
	l.elems.insert(l.elems.end(), r.elems.begin(), r.elems.end());
	return Indexed_Mesh(std::move(l.verts), std::move(l.elems));
}

LData merge(LData&& l, LData&& r) {
	l.verts.insert(l.verts.end(), r.verts.begin(), r.verts.end());
	return std::move(l);
}

LData circle(Spectrum color, float r, uint32_t sides) {

	std::vector<Vec3> points;
	float t = 0.0f;
	float step = (2.0f * PI_F) / (sides + 1);
	for (uint32_t i = 0; i < sides; i++) {
		points.push_back(r * Vec3(std::sin(t), 0.0f, std::cos(t)));
		t += step;
	}

	std::vector<GL::Lines::Vert> verts;
	for (size_t i = 0; i < points.size(); i++) {
		verts.push_back({points[i], color});
		verts.push_back({points[(i + 1) % points.size()], color});
	}

	return LData{std::move(verts)};
}

Data quad(float x, float y) {
	return {{{Vec3{-x, 0.0f, -y}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0},
	         {Vec3{-x, 0.0f, y}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 1.0f}, 1},
	         {Vec3{x, 0.0f, -y}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 2},
	         {Vec3{x, 0.0f, y}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{1.0f, 1.0f}, 3}},
	        {0, 1, 2, 2, 1, 3}};
}

Data cube(float r) {
	return {{{Vec3{-r, -r, -r}, Vec3{-r, -r, -r}.unit(), Vec2{0.0f, 0.0f}, 0},
	         {Vec3{r, -r, -r}, Vec3{r, -r, -r}.unit(), Vec2{1.0f, 0.0f}, 1},
	         {Vec3{r, r, -r}, Vec3{r, r, -r}.unit(), Vec2{1.0f, 1.0f}, 2},
	         {Vec3{-r, r, -r}, Vec3{-r, r, -r}.unit(), Vec2{0.0f, 1.0f}, 3},
	         {Vec3{-r, -r, r}, Vec3{-r, -r, r}.unit(), Vec2{0.0f, 0.0f}, 4},
	         {Vec3{r, -r, r}, Vec3{r, -r, r}.unit(), Vec2{1.0f,0.0f}, 5},
	         {Vec3{r, r, r}, Vec3{r, r, r}.unit(), Vec2{1.0f,1.0f}, 6},
	         {Vec3{-r, r, r}, Vec3{-r, r, r}.unit(), Vec2{0.0f,1.0f}, 7}},
	        {0, 1, 3, 3, 1, 2, 1, 5, 2, 2, 5, 6, 5, 4, 6, 6, 4, 7,
	         4, 0, 7, 7, 0, 3, 3, 2, 7, 7, 2, 6, 4, 5, 0, 0, 5, 1}};
}

// A0T3: Pentagon
// TODO: You have located the source of this problem. Remember your error 
//       message and fix the following function to generate a proper pentagon.
//       Note that the first vector of our indexed mesh constructor specifies 
//       the vertices this mesh contains. The second vector specifies faces to 
//       be constructed with these vertices, where every three vertex ID's form
//       a triangular face.
// Hint: You need not understand what halfedge meshes are or what exactly how 
//       the helper functions work for this task. Try drawing these vertices 
//       and faces on paper to see where the problem might lie.
Data pentagon(float r) {
	return {{{Vec3{r * std::cos(0.f * 2.f * PI_F / 5), 0.0f, r * std::sin(0.f * 2.f * PI_F / 5)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0},
	         {Vec3{r * std::cos(1.f * 2.f * PI_F / 5), 0.0f, r * std::sin(1.f * 2.f * PI_F / 5)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 1.0f}, 1},
	         {Vec3{r * std::cos(2.f * 2.f * PI_F / 5), 0.0f, r * std::sin(2.f * 2.f * PI_F / 5)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 2},
	         {Vec3{r * std::cos(3.f * 2.f * PI_F / 5), 0.0f, r * std::sin(3.f * 2.f * PI_F / 5)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{1.0f, 1.0f}, 3},
	         {Vec3{r * std::cos(4.f * 2.f * PI_F / 5), 0.0f, r * std::sin(4.f * 2.f * PI_F / 5)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{1.0f, 1.0f}, 4}},
	        {0, 2, 1, 0, 3, 2, 0, 4, 3}};
}

Data tetrahedron(float radius, float height) {
	// std::vector<Indexed_Mesh::Vert> verts;
	return {{{Vec3{0.0f, height, 0.0f}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 0.0f}, 0},
					{Vec3{radius * std::cos(0.f * 2.f * PI_F / 3), 0.0f, radius * std::sin(0.f * 2.f * PI_F / 3)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 0.0f}, 1},
	        {Vec3{radius * std::cos(1.f * 2.f * PI_F / 3), 0.0f, radius * std::sin(1.f * 2.f * PI_F / 3)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{0.0f, 1.0f}, 2},
	        {Vec3{radius * std::cos(2.f * 2.f * PI_F / 3), 0.0f, radius * std::sin(2.f * 2.f * PI_F / 3)}, Vec3{0.0f, 1.0f, 0.0f}, Vec2{1.0f, 0.0f}, 3}
					},
	        {0, 2, 1, 0, 3, 2, 0, 1, 3, 1, 2, 3}};
}

Data custom1(){
	return {{
					{Vec3{-0.139527f, 0.857635f, 0.024336f}, Vec3{-0.4296f, 0.8485f, -0.3091f}, Vec2{0,0},0}, 
					{Vec3{-0.166586f, 0.840026f, 0.027891f}, Vec3{-0.723045f, 0.62911f, -0.285355f}, Vec2{0,0},1}, 
					{Vec3{-0.193389f, 0.825913f, 0.070959f}, Vec3{-0.7656f, 0.6227f, -0.1618f}, Vec2{0,0},2}, 
					{Vec3{-0.143201f, 0.867156f, 0.065209f}, Vec3{-0.429f, 0.8868f, -0.1718f}, Vec2{0,0},3}, 
					{Vec3{-0.169969f, 0.849928f, 0.067663f}, Vec3{-0.632f, 0.7559f, -0.1712f}, Vec2{0,0},4}, 
					{Vec3{-0.111616f, 0.876789f, 0.061902f}, Vec3{-0.2228f, 0.9589f, -0.1759f}, Vec2{0,0},5}, 
					{Vec3{-0.158336f, 0.86002f, 0.0880435f}, Vec3{-0.540295f, 0.835001f, -0.104192f}, Vec2{0,0},6}, 
					{Vec3{-0.128897f, 0.873906f, 0.0846895f}, Vec3{-0.337808f, 0.934323f, -0.113694f}, Vec2{0,0},7}, 
					{Vec3{-0.154893f, 0.853591f, 0.04655f}, Vec3{-0.534307f, 0.812448f, -0.233333f}, Vec2{0,0},8}, 
					{Vec3{-0.18334f, 0.839377f, 0.0900985f}, Vec3{-0.705492f, 0.701566f, -0.100432f}, Vec2{0,0},9}, 
					{Vec3{-0.125571f, 0.867212f, 0.043119f}, Vec3{-0.329195f, 0.911997f, -0.244727f}, Vec2{0,0},10}, 
					{Vec3{-0.160806f, 0.860606f, 0.120416f}, Vec3{-0.684138f, 0.72405f, 0.0877886f}, Vec2{0,0},11}, 
					{Vec3{-0.185594f, 0.82411f, 0.0414257f}, Vec3{-0.782533f, 0.597695f, -0.174365f}, Vec2{0,0},12} 
					}, 
	        {12, 2, 4, 6, 3, 4, 7, 5, 3, 8, 4, 3, 9, 4, 2, 10, 3, 5, 1, 12, 4, 11, 6, 4, 3, 6, 11, 11, 7, 3, 4, 8, 1, 0, 8, 3, 4, 9, 11, 3, 10, 0, } 
				};
}

Data custom2(){
	return {{
					{Vec3{-0.212614f, 0.881316f, 0.8376f}, Vec3{-0.4442f, 0.1001f, 0.8903f}, Vec2{0,0}, 0}, 
					{Vec3{-0.236458f, 0.897004f, 0.817907f}, Vec3{-0.6402f, 0.2694f, 0.7195f}, Vec2{0,0}, 1}, 
					{Vec3{-0.0776756f, 1.04482f, 0.770222f}, Vec3{-0.037866f, 0.46346f, 0.885309f}, Vec2{0,0}, 2}, 
					{Vec3{-0.0268951f, 0.972775f, 0.819449f}, Vec3{0.0186004f, 0.492565f, 0.870077f}, Vec2{0,0}, 3}, 
					{Vec3{-0.106065f, 0.869331f, 0.856769f}, Vec3{0.0205018f, -0.106624f, 0.994088f}, Vec2{0,0}, 4}, 
					{Vec3{-0.213219f, 0.835368f, 0.828382f}, Vec3{-0.166204f, -0.680758f, 0.713403f}, Vec2{0,0}, 5}, 
					{Vec3{-0.262572f, 0.884888f, 0.762055f}, Vec3{-0.974508f, -0.0459791f, 0.219591f}, Vec2{0,0}, 6}, 
					{Vec3{-0.239239f, 1.01303f, 0.712297f}, Vec3{-0.897449f, 0.339228f, 0.281974f}, Vec2{0,0}, 7}, 
					{Vec3{-0.187191f, 1.10227f, 0.716392f}, Vec3{-0.62284f, 0.481377f, 0.616723f}, Vec2{0,0}, 8}, 
					{Vec3{-0.045222f, 1.02003f, 0.787077f}, Vec3{-0.111691f, 0.468594f, 0.876324f}, Vec2{0,0}, 9}, 
					{Vec3{-0.162928f, 0.994317f, 0.788254f}, Vec3{-0.390645f, 0.552131f, 0.736578f}, Vec2{0,0}, 10}, 
					{Vec3{-0.155762f, 0.943993f, 0.806139f}, Vec3{-0.3469f, 0.2867f, 0.893f}, Vec2{0,0}, 11}, 
					{Vec3{-0.185851f, 0.78929f, 0.784625f}, Vec3{-0.9962f, 0.0574f, 0.0662f}, Vec2{0,0}, 12}, 
					{Vec3{0.0791128f, 0.879565f, 0.847752f}, Vec3{-0.0201502f, 0.492722f, 0.869954f}, Vec2{0,0}, 13}, 
					}, 
	        {0, 1, 5, 11, 5, 1, 3, 11, 13, 3, 9, 11, 10, 7, 11, 2, 8, 11, 1, 0, 11, 0, 4, 11, 13, 11, 4, 11, 12, 5, 2, 11, 9, 8, 10, 11, 7, 6, 11, 12, 11, 6, } 
				};
}


// https://wiki.unity3d.com/index.php/ProceduralPrimitives
Data cone(float bradius, float tradius, float height, uint32_t sides, bool caps) {
// std::cout << "caps:" << caps << ", height:" << ", bradius: "<< bradius << height << std::endl;
// print_stacktrace();
	const uint32_t n_sides = sides, n_cap = n_sides + 1;
	constexpr float _2pi = PI_F * 2.0f;

	std::vector<Vec3> vertices(n_cap + n_cap + n_sides * 2 + 2);
	size_t vert = 0;

	vertices[vert++] = Vec3(0.0f, 0.0f, 0.0f); // 底面中心点(0,0,0)

	float t = 0.0f;
	float step = _2pi / n_sides;

	while (vert <= n_sides) {
		// 底面圆周上的顶点
		vertices[vert] = Vec3(std::cos(t) * bradius, 0.0f, std::sin(t) * bradius);
		vert++;
		t += step;
	}

	vertices[vert++] = Vec3(0.0f, height, 0.0f); // 顶面中心点
	t = 0.0f;
	while (vert <= n_sides * 2 + 1) {
		// 顶面圆周上的顶点
		vertices[vert] = Vec3(std::cos(t) * tradius, height, std::sin(t) * tradius);
		vert++;
		t += step;
	}

	uint32_t v = 0;
	t = 0.0f;
	while (vert <= vertices.size() - 4) {
		// 侧面顶点(连接顶面和底面)
		vertices[vert] = Vec3(std::cos(t) * tradius, height, std::sin(t) * tradius);
		vertices[vert + 1] = Vec3(std::cos(t) * bradius, 0.0f, std::sin(t) * bradius);
		vert += 2;
		v++;
		t += step;
	}
	vertices[vert] = vertices[n_sides * 2 + 2];
	vertices[vert + 1] = vertices[n_sides * 2 + 3];

	std::vector<Vec3> normals(vertices.size());
	vert = 0;
	while (vert <= n_sides) {
		// 底面法线全部向下
		normals[vert++] = Vec3(0.0f, -1.0f, 0.0f);
	}
	while (vert <= n_sides * 2 + 1) {
		// 顶面法线全部向上
		normals[vert++] = Vec3(0.0f, 1.0f, 0.0f);
	}

	v = 0;
	while (vert <= vertices.size() - 4) {
		// 侧面法线根据角度计算，垂直于侧面
		float rad = static_cast<float>(v) / n_sides * _2pi;
		float cos = std::cos(rad);
		float sin = std::sin(rad);
		normals[vert] = Vec3(cos, 0.0f, sin);
		normals[vert + 1] = normals[vert];
		vert += 2;
		v++;
	}
	normals[vert] = normals[n_sides * 2 + 2];
	normals[vert + 1] = normals[n_sides * 2 + 3];

	uint32_t n_tris = n_sides + n_sides + n_sides * 2;
	std::vector<Indexed_Mesh::Index> triangles(n_tris * 3 + 3);

	Indexed_Mesh::Index tri = 0;
	uint32_t i = 0;
	while (tri + 1 < n_sides) {
		if (caps) {
			triangles[i] = 0;
			triangles[i + 1] = tri + 1;
			triangles[i + 2] = tri + 2;
		}
		tri++;
		i += 3;
	}
	if (caps) {
		triangles[i] = 0;
		triangles[i + 1] = tri + 1;
		triangles[i + 2] = 1;
	}
	tri++;
	i += 3;

	while (tri < n_sides * 2) {
		if (caps) {
			triangles[i] = tri + 2;
			triangles[i + 1] = tri + 1;
			triangles[i + 2] = (GLuint)n_cap;
		}
		tri++;
		i += 3;
	}
	if (caps) {
		triangles[i] = (GLuint)n_cap + 1;
		triangles[i + 1] = tri + 1;
		triangles[i + 2] = (GLuint)n_cap;
	}
	tri++;
	i += 3;
	tri++;

	while (tri <= n_tris) {
		triangles[i] = tri + 2;
		triangles[i + 1] = tri + 1;
		triangles[i + 2] = tri + 0;
		tri++;
		i += 3;
		triangles[i] = tri + 1;
		triangles[i + 1] = tri + 2;
		triangles[i + 2] = tri + 0;
		tri++;
		i += 3;
	}

	std::vector<Indexed_Mesh::Vert> verts;
	for (size_t j = 0; j < vertices.size(); j++) {
		verts.push_back({vertices[j], normals[j], Vec2{}, static_cast<uint32_t>(j)});
	}
	return {verts, triangles};
}

Data torus(float iradius, float oradius, uint32_t segments, uint32_t sides) {

	const uint32_t n_rad_sides = segments, n_sides = sides;
	constexpr float _2pi = PI_F * 2.0f;
	iradius = oradius - iradius;

	std::vector<Vec3> vertices((n_rad_sides + 1) * (n_sides + 1));
	for (uint32_t seg = 0; seg <= n_rad_sides; seg++) {

		uint32_t cur_seg = seg == n_rad_sides ? 0 : seg;

		float t1 = static_cast<float>(cur_seg) / n_rad_sides * _2pi;
		Vec3 r1(std::cos(t1) * oradius, 0.0f, std::sin(t1) * oradius);

		for (uint32_t side = 0; side <= n_sides; side++) {

			uint32_t cur_side = side == n_sides ? 0 : side;
			float t2 = static_cast<float>(cur_side) / n_sides * _2pi;
			Vec3 r2 = Mat4::angle_axis(Degrees(-t1), Vec3{0.0f, 1.0f, 0.0f}) *
			          Vec3(std::sin(t2) * iradius, std::cos(t2) * iradius, 0.0f);

			vertices[side + seg * (n_sides + 1)] = r1 + r2;
		}
	}

	std::vector<Vec3> normals(vertices.size());
	for (uint32_t seg = 0; seg <= n_rad_sides; seg++) {

		uint32_t cur_seg = seg == n_rad_sides ? 0 : seg;
		float t1 = static_cast<float>(cur_seg) / n_rad_sides * _2pi;
		Vec3 r1(std::cos(t1) * oradius, 0.0f, std::sin(t1) * oradius);

		for (uint32_t side = 0; side <= n_sides; side++) {
			normals[side + seg * (n_sides + 1)] =
				(vertices[side + seg * (n_sides + 1)] - r1).unit();
		}
	}

	uint32_t n_faces = static_cast<uint32_t>(vertices.size());
	uint32_t n_tris = n_faces * 2;
	uint32_t n_idx = n_tris * 3;
	std::vector<Indexed_Mesh::Index> triangles(n_idx);

	uint32_t i = 0;
	for (uint32_t seg = 0; seg <= n_rad_sides; seg++) {
		for (uint32_t side = 0; side < n_sides; side++) {

			uint32_t current = side + seg * (n_sides + 1);
			uint32_t next = side + (seg < (n_rad_sides) ? (seg + 1) * (n_sides + 1) : 0);

			if (i < triangles.size() - 6) {
				triangles[i++] = current;
				triangles[i++] = next;
				triangles[i++] = next + 1;
				triangles[i++] = current;
				triangles[i++] = next + 1;
				triangles[i++] = current + 1;
			}
		}
	}

	std::vector<Indexed_Mesh::Vert> verts;
	for (size_t j = 0; j < vertices.size(); j++) {
		verts.push_back({vertices[j], normals[j], Vec2{}, static_cast<uint32_t>(j)});
	}
	return {verts, triangles};
}

Data uv_hemisphere(float radius) {

	constexpr uint32_t nbLong = 64;
	constexpr uint32_t nbLat = 16;

	std::vector<Vec3> vertices((nbLong + 1) * nbLat + 2);
	float _pi = PI_F;
	float _2pi = _pi * 2.0f;

	vertices[0] = Vec3{0.0f, radius, 0.0f};
	for (uint32_t lat = 0; lat < nbLat; lat++) {
		float a1 = _pi * static_cast<float>(lat + 1) / (nbLat + 1);
		float sin1 = std::sin(a1);
		float cos1 = std::cos(a1);

		for (uint32_t lon = 0; lon <= nbLong; lon++) {
			float a2 = _2pi * static_cast<float>(lon == nbLong ? 0 : lon) / nbLong;
			float sin2 = std::sin(a2);
			float cos2 = std::cos(a2);

			vertices[lon + lat * (nbLong + 1) + 1] = Vec3(sin1 * cos2, cos1, sin1 * sin2) * radius;
		}
	}
	vertices[vertices.size() - 1] = Vec3{0.0f, -radius, 0.0f};

	std::vector<Vec3> normals(vertices.size());
	for (size_t n = 0; n < vertices.size(); n++) normals[n] = vertices[n].unit();

	uint32_t nbFaces = static_cast<uint32_t>(vertices.size());
	uint32_t nbTriangles = nbFaces * 2;
	uint32_t nbIndexes = nbTriangles * 3;
	std::vector<Indexed_Mesh::Index> triangles(nbIndexes);

	uint32_t i = 0;
	for (uint32_t lat = (nbLat - 1) / 2; lat < nbLat - 1; lat++) {
		for (uint32_t lon = 0; lon < nbLong; lon++) {
			uint32_t current = lon + lat * (nbLong + 1) + 1;
			uint32_t next = current + nbLong + 1;

			triangles[i++] = current;
			triangles[i++] = current + 1;
			triangles[i++] = next + 1;

			triangles[i++] = current;
			triangles[i++] = next + 1;
			triangles[i++] = next;
		}
	}

	for (uint32_t lon = 0; lon < nbLong; lon++) {
		uint32_t size = static_cast<uint32_t>(vertices.size());
		triangles[i++] = size - 1;
		triangles[i++] = size - (lon + 2) - 1;
		triangles[i++] = size - (lon + 1) - 1;
	}

	std::vector<Indexed_Mesh::Vert> verts;
	for (size_t j = 0; j < vertices.size(); j++) {
		verts.push_back({vertices[j], normals[j], Vec2{}, static_cast<uint32_t>(j)});
	}
	triangles.resize(i);
	return {verts, triangles};
}

Data texture_ico_sphere(float radius, uint32_t level) {
	struct TriIdx {
		uint32_t v1, v2, v3;
	};

	auto middle_point = [&](uint32_t p1, uint32_t p2, std::vector<Vec3>& vertices,
	                        std::map<int64_t, uint32_t>& cache, float radius) -> uint32_t {
		bool firstIsSmaller = p1 < p2;
		int64_t smallerIndex = firstIsSmaller ? p1 : p2;
		int64_t greaterIndex = firstIsSmaller ? p2 : p1;
		int64_t key = (smallerIndex << 32ll) + greaterIndex;

		auto entry = cache.find(key);
		if (entry != cache.end()) {
			return entry->second;
		}

		Vec3 point1 = vertices[p1];
		Vec3 point2 = vertices[p2];
		Vec3 middle((point1.x + point2.x) / 2.0f, (point1.y + point2.y) / 2.0f,
		            (point1.z + point2.z) / 2.0f);
		uint32_t i = static_cast<uint32_t>(vertices.size());
		vertices.push_back(middle.unit() * radius);
		cache[key] = i;
		return i;
	};

	std::vector<Vec3> vertices;
	std::map<int64_t, uint32_t> middlePointIndexCache;
	float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
	vertices.push_back(Vec3(-1.0f, t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(1.0f, t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(-1.0f, -t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(1.0f, -t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(0.0f, -1.0f, t).unit() * radius);
	vertices.push_back(Vec3(0.0f, 1.0f, t).unit() * radius);
	vertices.push_back(Vec3(0.0f, -1.0f, -t).unit() * radius);
	vertices.push_back(Vec3(0.0f, 1.0f, -t).unit() * radius);
	vertices.push_back(Vec3(t, 0.0f, -1.0f).unit() * radius);
	vertices.push_back(Vec3(t, 0.0f, 1.0f).unit() * radius);
	vertices.push_back(Vec3(-t, 0.0f, -1.0f).unit() * radius);
	vertices.push_back(Vec3(-t, 0.0f, 1.0f).unit() * radius);

	std::vector<TriIdx> faces;
	faces.push_back(TriIdx{0, 11, 5});
	faces.push_back(TriIdx{0, 5, 1});
	faces.push_back(TriIdx{0, 1, 7});
	faces.push_back(TriIdx{0, 7, 10});
	faces.push_back(TriIdx{0, 10, 11});
	faces.push_back(TriIdx{1, 5, 9});
	faces.push_back(TriIdx{5, 11, 4});
	faces.push_back(TriIdx{11, 10, 2});
	faces.push_back(TriIdx{10, 7, 6});
	faces.push_back(TriIdx{7, 1, 8});
	faces.push_back(TriIdx{3, 9, 4});
	faces.push_back(TriIdx{3, 4, 2});
	faces.push_back(TriIdx{3, 2, 6});
	faces.push_back(TriIdx{3, 6, 8});
	faces.push_back(TriIdx{3, 8, 9});
	faces.push_back(TriIdx{4, 9, 5});
	faces.push_back(TriIdx{2, 4, 11});
	faces.push_back(TriIdx{6, 2, 10});
	faces.push_back(TriIdx{8, 6, 7});
	faces.push_back(TriIdx{9, 8, 1});

	for (uint32_t i = 0; i < level; i++) {
		std::vector<TriIdx> faces2;
		for (auto tri : faces) {
			uint32_t a = middle_point(tri.v1, tri.v2, vertices, middlePointIndexCache, radius);
			uint32_t b = middle_point(tri.v2, tri.v3, vertices, middlePointIndexCache, radius);
			uint32_t c = middle_point(tri.v3, tri.v1, vertices, middlePointIndexCache, radius);
			faces2.push_back(TriIdx{tri.v1, a, c});
			faces2.push_back(TriIdx{tri.v2, b, a});
			faces2.push_back(TriIdx{tri.v3, c, b});
			faces2.push_back(TriIdx{a, b, c});
		}
		faces = faces2;
	}

	std::vector<Vec3> normals(vertices.size());
	for (size_t i = 0; i < normals.size(); i++) normals[i] = vertices[i].unit();

	std::vector<Vec2> uvs(vertices.size());
	for (size_t i = 0; i < uvs.size(); i++) {
		Vec3 dir = vertices[i].unit();
		uvs[i] = Shapes::Sphere::uv(dir);
	}

	// Try to fix UVs, following https://mft-dev.dk/uv-mapping-sphere/
	// Detect triangles with wrong winding order
	std::vector<uint32_t> winding_indices;
	for (size_t i = 0; i < faces.size(); i++) {
		uint32_t v1 = faces[i].v1;
		uint32_t v2 = faces[i].v2;
		uint32_t v3 = faces[i].v3;
		Vec3 tex_v1 = Vec3(uvs[v1].x, uvs[v1].y, 0.0f);
		Vec3 tex_v2 = Vec3(uvs[v2].x, uvs[v2].y, 0.0f);
		Vec3 tex_v3 = Vec3(uvs[v3].x, uvs[v3].y, 0.0f);
		Vec3 tex_normal = cross(tex_v2 - tex_v1, tex_v3 - tex_v1);
		if (tex_normal.z > 0) {
			winding_indices.push_back(static_cast<uint32_t>(i));
		}
	}
	// Fix these vertices
	auto fix_zig_zag = [&](std::vector<Vec3>& vertices, std::vector<Vec3>& normals,
	                       std::vector<Vec2>& uvs, std::unordered_map<uint32_t, uint32_t>& visited,
	                       std::set<uint32_t>& cap_vertices, size_t& vertex_index,
	                       uint32_t& vertex) -> void {
		if (uvs[vertex].x < 0.25f) {
			uint32_t temp_vertex = vertex;
			if (!visited.count(vertex)) {
				// Create a copy of the vertex but with uv changed
				Vec3 vertex_copy = vertices[vertex];
				Vec3 normal_copy = normals[vertex];
				Vec2 uv_copy = uvs[vertex];
				uv_copy.x += 1;
				// Add to the list of vertices
				vertices.push_back(vertex_copy);
				normals.push_back(normal_copy);
				uvs.push_back(uv_copy);
				// Update faces
				vertex_index++;
				if (std::abs(vertex_copy.y - 1) < 0.001f || std::abs(vertex_copy.y + 1) < 0.001f) {
					cap_vertices.insert(static_cast<uint32_t>(vertex_index));
				}
				visited.insert({vertex, static_cast<uint32_t>(vertex_index)});
				temp_vertex = static_cast<uint32_t>(vertex_index);
			} else {
				temp_vertex = visited.at(vertex);
			}
			vertex = temp_vertex;
		}
	};
	std::set<uint32_t> cap_vertices;
	size_t vertex_index = vertices.size() - 1;
	std::unordered_map<uint32_t, uint32_t> visited;
	for (auto& index : winding_indices) {
		uint32_t v1 = faces[index].v1;
		uint32_t v2 = faces[index].v2;
		uint32_t v3 = faces[index].v3;
		fix_zig_zag(vertices, normals, uvs, visited, cap_vertices, vertex_index, v1);
		fix_zig_zag(vertices, normals, uvs, visited, cap_vertices, vertex_index, v2);
		fix_zig_zag(vertices, normals, uvs, visited, cap_vertices, vertex_index, v3);
		faces[index].v1 = v1;
		faces[index].v2 = v2;
		faces[index].v3 = v3;
	}

	// Fix top cap
	auto fix_cap = [&](std::vector<Vec3>& vertices, std::vector<Vec3>& normals,
	                   std::vector<Vec2>& uvs, std::vector<TriIdx>& faces, size_t& vertex_index,
	                   bool& create_new_vertex, size_t index, uint32_t cap_index) -> void {
		if (create_new_vertex) {
			if (faces[index].v1 == cap_index) {
				// Create a copy of the vertex but with uv changed
				Vec3 vertex_copy = vertices[cap_index];
				Vec3 normal_copy = normals[cap_index];
				Vec2 uv_copy = uvs[cap_index];
				uv_copy.x = (uvs[faces[index].v2].x + uvs[faces[index].v3].x) / 2.0f;
				// Add to the list of vertices
				vertices.push_back(vertex_copy);
				normals.push_back(normal_copy);
				uvs.push_back(uv_copy);
				// Update faces
				vertex_index++;
				faces[index].v1 = static_cast<uint32_t>(vertex_index);
			} else if (faces[index].v2 == cap_index) {
				// Create a copy of the vertex but with uv changed
				Vec3 vertex_copy = vertices[cap_index];
				Vec3 normal_copy = normals[cap_index];
				Vec2 uv_copy = uvs[cap_index];
				uv_copy.x = (uvs[faces[index].v1].x + uvs[faces[index].v3].x) / 2.0f;
				// Add to the list of vertices
				vertices.push_back(vertex_copy);
				normals.push_back(normal_copy);
				uvs.push_back(uv_copy);
				// Update faces
				vertex_index++;
				faces[index].v2 = static_cast<uint32_t>(vertex_index);
			} else if (faces[index].v3 == cap_index) {
				// Create a copy of the vertex but with uv changed
				Vec3 vertex_copy = vertices[cap_index];
				Vec3 normal_copy = normals[cap_index];
				Vec2 uv_copy = uvs[cap_index];
				uv_copy.x = (uvs[faces[index].v1].x + uvs[faces[index].v2].x) / 2.0f;
				// Add to the list of vertices
				vertices.push_back(vertex_copy);
				normals.push_back(normal_copy);
				uvs.push_back(uv_copy);
				// Update faces
				vertex_index++;
				faces[index].v3 = static_cast<uint32_t>(vertex_index);
			}
		} else {
			if (faces[index].v1 == cap_index) {
				uvs[cap_index].x = (uvs[faces[index].v2].x + uvs[faces[index].v3].x) / 2.0f;
				create_new_vertex = true;
			} else if (faces[index].v2 == cap_index) {
				uvs[cap_index].x = (uvs[faces[index].v1].x + uvs[faces[index].v3].x) / 2.0f;
				create_new_vertex = true;
			} else if (faces[index].v3 == cap_index) {
				uvs[cap_index].x = (uvs[faces[index].v1].x + uvs[faces[index].v2].x) / 2.0f;
				create_new_vertex = true;
			}
		}
	};
	std::vector<uint32_t> north_indices = {};
	std::vector<uint32_t> south_indices = {};
	for (size_t i = 0; i < vertices.size(); i++) {
		if (std::abs(vertices[i].y - 1) < 0.001f && !cap_vertices.count(static_cast<uint32_t>(i))) {
			north_indices.push_back(static_cast<uint32_t>(i));
		} else if (std::abs(vertices[i].y + 1) < 0.001f &&
		           !cap_vertices.count(static_cast<uint32_t>(i))) {
			south_indices.push_back(static_cast<uint32_t>(i));
		}
	}
	vertex_index = vertices.size() - 1;
	bool create_north_vertex = false;
	bool create_south_vertex = false;
	for (size_t i = 0; i < faces.size(); i++) {
		for (size_t j = 0; j < north_indices.size(); j++) {
			fix_cap(vertices, normals, uvs, faces, vertex_index, create_north_vertex, i,
			        north_indices[j]);
		}
		for (size_t j = 0; j < south_indices.size(); j++) {
			fix_cap(vertices, normals, uvs, faces, vertex_index, create_south_vertex, i,
			        south_indices[j]);
		}
	}

	// Construct the indexed mesh
	std::vector<Indexed_Mesh::Index> triangles;
	for (size_t i = 0; i < faces.size(); i++) {
		triangles.push_back(faces[i].v1);
		triangles.push_back(faces[i].v2);
		triangles.push_back(faces[i].v3);
	}
	std::vector<Indexed_Mesh::Vert> verts;
	for (size_t i = 0; i < vertices.size(); i++) {
		verts.push_back({vertices[i], normals[i], uvs[i], static_cast<uint32_t>(i)});
	}
	return {verts, triangles};
}

Data closed_ico_sphere(float radius, uint32_t level) {
	struct TriIdx {
		uint32_t v1, v2, v3;
	};

	auto middle_point = [&](uint32_t p1, uint32_t p2, std::vector<Vec3>& vertices,
	                        std::map<int64_t, uint32_t>& cache, float radius) -> uint32_t {
		bool firstIsSmaller = p1 < p2;
		int64_t smallerIndex = firstIsSmaller ? p1 : p2;
		int64_t greaterIndex = firstIsSmaller ? p2 : p1;
		int64_t key = (smallerIndex << 32ll) + greaterIndex;

		auto entry = cache.find(key);
		if (entry != cache.end()) {
			return entry->second;
		}

		Vec3 point1 = vertices[p1];
		Vec3 point2 = vertices[p2];
		Vec3 middle((point1.x + point2.x) / 2.0f, (point1.y + point2.y) / 2.0f,
		            (point1.z + point2.z) / 2.0f);
		uint32_t i = static_cast<uint32_t>(vertices.size());
		vertices.push_back(middle.unit() * radius);
		cache[key] = i;
		return i;
	};

	std::vector<Vec3> vertices;
	std::map<int64_t, uint32_t> middlePointIndexCache;
	float t = (1.0f + std::sqrt(5.0f)) / 2.0f;
	vertices.push_back(Vec3(-1.0f, t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(1.0f, t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(-1.0f, -t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(1.0f, -t, 0.0f).unit() * radius);
	vertices.push_back(Vec3(0.0f, -1.0f, t).unit() * radius);
	vertices.push_back(Vec3(0.0f, 1.0f, t).unit() * radius);
	vertices.push_back(Vec3(0.0f, -1.0f, -t).unit() * radius);
	vertices.push_back(Vec3(0.0f, 1.0f, -t).unit() * radius);
	vertices.push_back(Vec3(t, 0.0f, -1.0f).unit() * radius);
	vertices.push_back(Vec3(t, 0.0f, 1.0f).unit() * radius);
	vertices.push_back(Vec3(-t, 0.0f, -1.0f).unit() * radius);
	vertices.push_back(Vec3(-t, 0.0f, 1.0f).unit() * radius);

	std::vector<TriIdx> faces;
	faces.push_back(TriIdx{0, 11, 5});
	faces.push_back(TriIdx{0, 5, 1});
	faces.push_back(TriIdx{0, 1, 7});
	faces.push_back(TriIdx{0, 7, 10});
	faces.push_back(TriIdx{0, 10, 11});
	faces.push_back(TriIdx{1, 5, 9});
	faces.push_back(TriIdx{5, 11, 4});
	faces.push_back(TriIdx{11, 10, 2});
	faces.push_back(TriIdx{10, 7, 6});
	faces.push_back(TriIdx{7, 1, 8});
	faces.push_back(TriIdx{3, 9, 4});
	faces.push_back(TriIdx{3, 4, 2});
	faces.push_back(TriIdx{3, 2, 6});
	faces.push_back(TriIdx{3, 6, 8});
	faces.push_back(TriIdx{3, 8, 9});
	faces.push_back(TriIdx{4, 9, 5});
	faces.push_back(TriIdx{2, 4, 11});
	faces.push_back(TriIdx{6, 2, 10});
	faces.push_back(TriIdx{8, 6, 7});
	faces.push_back(TriIdx{9, 8, 1});

	for (uint32_t i = 0; i < level; i++) {
		std::vector<TriIdx> faces2;
		for (auto tri : faces) {
			uint32_t a = middle_point(tri.v1, tri.v2, vertices, middlePointIndexCache, radius);
			uint32_t b = middle_point(tri.v2, tri.v3, vertices, middlePointIndexCache, radius);
			uint32_t c = middle_point(tri.v3, tri.v1, vertices, middlePointIndexCache, radius);
			faces2.push_back(TriIdx{tri.v1, a, c});
			faces2.push_back(TriIdx{tri.v2, b, a});
			faces2.push_back(TriIdx{tri.v3, c, b});
			faces2.push_back(TriIdx{a, b, c});
		}
		faces = faces2;
	}

	std::vector<Indexed_Mesh::Index> triangles;
	for (size_t i = 0; i < faces.size(); i++) {
		triangles.push_back(faces[i].v1);
		triangles.push_back(faces[i].v2);
		triangles.push_back(faces[i].v3);
	}

	std::vector<Vec3> normals(vertices.size());
	for (size_t i = 0; i < normals.size(); i++) normals[i] = vertices[i].unit();

	std::vector<Vec2> uvs(vertices.size());
	for (size_t i = 0; i < uvs.size(); i++) {
		Vec3 dir = vertices[i].unit();
		uvs[i] = Shapes::Sphere::uv(dir);
	}

	std::vector<Indexed_Mesh::Vert> verts;
	for (size_t i = 0; i < vertices.size(); i++) {
		verts.push_back({vertices[i], normals[i], uvs[i], 0});
	}
	return {verts, triangles};
}

} // namespace Gen
} // namespace Util
