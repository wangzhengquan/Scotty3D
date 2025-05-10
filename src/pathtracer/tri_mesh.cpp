
#include "../test.h"

#include "samplers.h"
#include "tri_mesh.h"
#include <iostream>

namespace PT {

/**
 * Compute the bounding box of the triangle.
*/
BBox Triangle::bbox() const {
  //A3T2 / A3T3

  // Beware of flat/zero-volume boxes! You may need to
  // account for that here, or later on in BBox::hit.
  BBox box;
   
  box.enclose(vertex_list[v0].position);
  box.enclose(vertex_list[v1].position);
  box.enclose(vertex_list[v2].position);
  // Ensure non-zero volume
  if (box.min == box.max) {
      box.max += Vec3(1e-5f);
  }
  return box;
}
 
/**
 * https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection.html
*/
Trace Triangle::hit(const Ray& ray) const {
  //A3T2
  // Get triangle vertices
  Tri_Mesh_Vert v_0 = vertex_list[v0];
  Tri_Mesh_Vert v_1 = vertex_list[v1];
  Tri_Mesh_Vert v_2 = vertex_list[v2];

  Trace ret;
  ret.origin = ray.point;
  ret.hit = false;

  // Möller-Trumbore algorithm
  Vec3 e1 = v_1.position - v_0.position;
  Vec3 e2 = v_2.position - v_0.position;
  Vec3 s = ray.point - v_0.position;
  Vec3 s1 = cross(ray.dir, e2);
  Vec3 s2 = cross(s, e1);

  float divisor = dot(s1, e1);
  if (divisor == 0.0f) return ret; // Ray parallel to triangle plane
// std::cout << "divisor: " << divisor << ", " <<  dot(cross(ray.dir, e1), e2) << std::endl;
  // Compute barycentric coordinates
  float u = dot(s1, s) / divisor;
  float v = dot(s2, ray.dir) / divisor;
  float t = dot(s2, e2) / divisor;

// std::cout << "\nu: " << u << ", " << dot(s1, s) << ", " <<  dot(cross(e2, s), ray.dir) << std::endl;
// std::cout << "\nv: " << v << ", " << dot(s2, ray.dir)  << ", " <<  dot(cross(e1, ray.dir), s) << std::endl;
// std::cout << "\nt: " << t << ", " << dot(s2, e2)  << ", " <<  dot(-cross(s, e2), e1) << std::endl;

  // Check if intersection is within triangle and ray bounds
  if (u >= 0.0f && v >= 0.0f && (u + v) <= 1.0f && 
    t >= ray.dist_bounds.x && t <= ray.dist_bounds.y) {
    
    ret.hit = true;
    ret.distance = t;
    ret.position = ray.at(t);
    
    // Interpolate normal
    float w = 1.0f - u - v;
    ret.normal = (w * v_0.normal + u * v_1.normal + v * v_2.normal).unit();
    
    // Interpolate UV coordinates 
    ret.uv = w * v_0.uv + u * v_1.uv + v * v_2.uv;
  }
  return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, uint32_t v0, uint32_t v1, uint32_t v2)
  : v0(v0), v1(v1), v2(v2), vertex_list(verts) {
}

Vec3 Triangle::sample(RNG &rng, Vec3 from) const {
  Tri_Mesh_Vert v_0 = vertex_list[v0];
  Tri_Mesh_Vert v_1 = vertex_list[v1];
  Tri_Mesh_Vert v_2 = vertex_list[v2];
  Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
  Vec3 pos = sampler.sample(rng);
  return (pos - from).unit();
}

float Triangle::pdf(Ray wray, const Mat4& T, const Mat4& iT) const {

  Ray tray = wray;
  tray.transform(iT);

  Trace trace = hit(tray);
  if (trace.hit) {
    trace.transform(T, iT.T());
    Vec3 v_0 = T * vertex_list[v0].position;
    Vec3 v_1 = T * vertex_list[v1].position;
    Vec3 v_2 = T * vertex_list[v2].position;
    Samplers::Triangle sampler(v_0, v_1, v_2);
    float a = sampler.pdf(trace.position);
    float g = (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
    return a * g;
  }
  return 0.0f;
}

bool Triangle::operator==(const Triangle& rhs) const {
  if (Test::differs(vertex_list[v0].position, rhs.vertex_list[rhs.v0].position) ||
      Test::differs(vertex_list[v0].normal, rhs.vertex_list[rhs.v0].normal) ||
      Test::differs(vertex_list[v0].uv, rhs.vertex_list[rhs.v0].uv) ||
      Test::differs(vertex_list[v1].position, rhs.vertex_list[rhs.v1].position) ||
      Test::differs(vertex_list[v1].normal, rhs.vertex_list[rhs.v1].normal) ||
      Test::differs(vertex_list[v1].uv, rhs.vertex_list[rhs.v1].uv) ||
      Test::differs(vertex_list[v2].position, rhs.vertex_list[rhs.v2].position) ||
      Test::differs(vertex_list[v2].normal, rhs.vertex_list[rhs.v2].normal) ||
      Test::differs(vertex_list[v2].uv, rhs.vertex_list[rhs.v2].uv)) {
    return false;
  }
  return true;
}

Tri_Mesh::Tri_Mesh(const Indexed_Mesh& mesh, bool use_bvh_) : use_bvh(use_bvh_) {
  for (const auto& v : mesh.vertices()) {
    verts.push_back({v.pos, v.norm, v.uv});
  }

  const auto& idxs = mesh.indices();

  std::vector<Triangle> tris;
  for (size_t i = 0; i < idxs.size(); i += 3) {
    tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
  }

  if (use_bvh) {
    triangle_bvh.build(std::move(tris), 4);
  } else {
    triangle_list = List<Triangle>(std::move(tris));
  }
}

Tri_Mesh Tri_Mesh::copy() const {
  Tri_Mesh ret;
  ret.verts = verts;
  ret.triangle_bvh = triangle_bvh.copy();
  ret.triangle_list = triangle_list.copy();
  ret.use_bvh = use_bvh;
  return ret;
}

BBox Tri_Mesh::bbox() const {
  if (use_bvh) return triangle_bvh.bbox();
  return triangle_list.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
  if (use_bvh) return triangle_bvh.hit(ray);
  return triangle_list.hit(ray);
}

size_t Tri_Mesh::n_triangles() const {
  return use_bvh ? triangle_bvh.n_primitives() : triangle_list.n_primitives();
}

uint32_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                             const Mat4& trans) const {
  if (use_bvh) return triangle_bvh.visualize(lines, active, level, trans);
  return 0u;
}

Vec3 Tri_Mesh::sample(RNG &rng, Vec3 from) const {
  if (use_bvh) {
    return triangle_bvh.sample(rng, from);
  }
  return triangle_list.sample(rng, from);
}

float Tri_Mesh::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
  if (use_bvh) {
    return triangle_bvh.pdf(ray, T, iT);
  }
  return triangle_list.pdf(ray, T, iT);
}

} // namespace PT
