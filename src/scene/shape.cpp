
#include "shape.h"
#include "../geometry/util.h"

namespace Shapes {

Vec2 Sphere::uv(Vec3 dir) {
	float u = std::atan2(dir.z, dir.x) / (2.0f * PI_F);
	if (u < 0.0f) u += 1.0f;
	float v = std::acos(-1.0f * std::clamp(dir.y, -1.0f, 1.0f)) / PI_F;
	return Vec2{u, v};
}

BBox Sphere::bbox() const {
	BBox box;
	box.enclose(Vec3(-radius));
	box.enclose(Vec3(radius));
	return box;
}

PT::Trace Shapes::Sphere::hit(Ray ray) const {
	//A3T2 - sphere hit

	// Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

	// If the ray intersects the sphere twice, ret should
	// represent the first intersection, but remember to respect
	// ray.dist_bounds! For example, if there are two intersections,
	// but only the _later_ one is within ray.dist_bounds, you should
	// return that one!

	PT::Trace ret;
	ret.origin = ray.point;
	ret.hit = false;

	// Quadratic equation coefficients
	Vec3 oc = ray.point; // Sphere center is at (0,0,0) in local space
	float a = ray.dir.norm_squared();
	float b = 2.0f * dot(oc, ray.dir);
	float c = oc.norm_squared() - radius * radius;

	// Discriminant
	float discriminant = b * b - 4 * a * c;
	if (discriminant < 0.0f) return ret; // No intersection

	float sqrt_discriminant = std::sqrt(discriminant);
	float t1 = (-b - sqrt_discriminant) / (2.0f * a);
	float t2 = (-b + sqrt_discriminant) / (2.0f * a);

	// Find the closest valid intersection
	float t = t1;
	if (t < ray.dist_bounds.x || t > ray.dist_bounds.y) {
			t = t2;
			if (t < ray.dist_bounds.x || t > ray.dist_bounds.y) {
					return ret; // Both intersections out of bounds
			}
	}

	ret.hit = true;
	ret.distance = t;
	ret.position = ray.at(t);
	ret.normal = ret.position.unit(); // Normal points outward
	
	// Compute UV coordinates using spherical mapping
	ret.uv = Sphere::uv(ret.normal);

	return ret;
}

Vec3 Sphere::sample(RNG &rng, Vec3 from) const {
	die("Sampling sphere area lights is not implemented yet.");
}

float Sphere::pdf(Ray ray, Mat4 pdf_T, Mat4 pdf_iT) const {
	die("Sampling sphere area lights is not implemented yet.");
}

Indexed_Mesh Sphere::to_mesh() const {
	return Util::closed_sphere_mesh(radius, 2);
}

} // namespace Shapes

bool operator!=(const Shapes::Sphere& a, const Shapes::Sphere& b) {
	return a.radius != b.radius;
}

bool operator!=(const Shape& a, const Shape& b) {
	if (a.shape.index() != b.shape.index()) return false;
	return std::visit(
		[&](const auto& shape) {
			return shape != std::get<std::decay_t<decltype(shape)>>(b.shape);
		},
		a.shape);
}
