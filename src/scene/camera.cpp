#include "camera.h"
#include "../gui/manager.h"
#include "../pathtracer/samplers.h"
#include "../test.h"


std::pair<Ray, float> Camera::sample_ray(RNG &rng, uint32_t px, uint32_t py) {
	//A3T1 - step 1 - camera rays
	//Sample a ray that starts at the origin and passes through pixel (px,py) + random offset on the sensor plane.
	//
	//Because cameras look down the -z axis, the "sensor plane" is
	// the rectangle from (-w/2,-h/2,-1) to (w/2,h/2,-1)
	// where:
	//  h is such that the angle made by (0,-h/2,-1) - (0,0,0) - (0,h/2,-1) is `vertical_fov`
	//  and w / h is given by `aspect_ratio`.
	//
	//The relationship between sensor pixels and the sensor plane is such that
	//  sensor pixel location (0,0) maps to (-w/2,-h/2,-1),
	//  and sensor pixel location (film.width,film.height) maps to (w/2,h/2,-1).

	// Sample random offset within pixel
	// --- 1. 像素内抗锯齿采样（始终用 Rect）---
	Samplers::Rect pixel_sampler; // 单位矩形采样器
	Vec2 offset = pixel_sampler.sample(rng);
	float offset_pdf = pixel_sampler.pdf(offset); // 固定为 1.0

	// --- 2. 计算传感器平面坐标 ---
	float h = 2.0f * std::tan(Radians(vertical_fov) / 2.0f);
	float w = aspect_ratio * h;
	Vec3 sensor_point(
			((px + offset.x) / film.width - 0.5f) * w * focal_dist,
			((py + offset.y) / film.height - 0.5f) * h * focal_dist,
			-focal_dist
	);

	// --- 3. 光圈采样（根据形状切换采样器）---
	Vec3 ray_origin(0.0f, 0.0f, 0.0f);
	float aperture_pdf = 1.0f; // 默认值（无光圈时）
	
	if (aperture_size > 0.0f) {
		if (aperture_shape == 2) { 
				// 圆形光圈采样
				Samplers::Circle circle_sampler(Vec2(), aperture_size / 2.0f);
				Vec2 aperture_sample = circle_sampler.sample(rng);
				ray_origin = Vec3(aperture_sample.x, aperture_sample.y, 0.0f);
				aperture_pdf = circle_sampler.pdf(aperture_sample); // 1/(πr²)
		} else {
				// 矩形光圈采样
				Samplers::Rect rect_sampler(Vec2(aperture_size, aperture_size));
				Vec2 aperture_sample = rect_sampler.sample(rng) - Vec2(aperture_size / 2.0f);
				ray_origin = Vec3(aperture_sample.x, aperture_sample.y, 0.0f);
				aperture_pdf = rect_sampler.pdf(aperture_sample + Vec2(aperture_size / 2.0f)); // 1/(aperture_size²)
		}
	}

	// --- 4. 构建光线 ---
	Ray ray;
	ray.point = ray_origin;
	ray.dir = (sensor_point - ray_origin).unit();
	ray.depth = film.max_ray_depth;

	// --- 5. 联合概率密度计算 ---
	float total_pdf = offset_pdf * aperture_pdf;
	return {ray, total_pdf};
}


Mat4 Camera::projection() const {
	return Mat4::perspective(vertical_fov, aspect_ratio, near_plane);
}

GL::Lines Camera::to_gl() const {

	GL::Lines cage;

	// float ap = near_plane * std::tan(Radians(vertical_fov) / 2.0f);
	float h = 2.0f * std::tan(Radians(vertical_fov) / 2.0f);
	float w = aspect_ratio * h;

	Vec3 tr = Vec3(0.5f * w, 0.5f * h, -1.0f) * focal_dist;
	Vec3 tl = Vec3(-0.5f * w, 0.5f * h, -1.0f) * focal_dist;
	Vec3 br = Vec3(0.5f * w, -0.5f * h, -1.0f) * focal_dist;
	Vec3 bl = Vec3(-0.5f * w, -0.5f * h, -1.0f) * focal_dist;

	Vec3 ftr = Vec3(0.5f * aperture_size, 0.5f * aperture_size, -near_plane);
	Vec3 ftl = Vec3(-0.5f * aperture_size, 0.5f * aperture_size, -near_plane);
	Vec3 fbr = Vec3(0.5f * aperture_size, -0.5f * aperture_size, -near_plane);
	Vec3 fbl = Vec3(-0.5f * aperture_size, -0.5f * aperture_size, -near_plane);

	cage.add(ftl, ftr, Gui::Color::black);
	cage.add(ftr, fbr, Gui::Color::black);
	cage.add(fbr, fbl, Gui::Color::black);
	cage.add(fbl, ftl, Gui::Color::black);

	cage.add(ftr, tr, Gui::Color::black);
	cage.add(ftl, tl, Gui::Color::black);
	cage.add(fbr, br, Gui::Color::black);
	cage.add(fbl, bl, Gui::Color::black);

	cage.add(bl, tl, Gui::Color::black);
	cage.add(tl, tr, Gui::Color::black);
	cage.add(tr, br, Gui::Color::black);
	cage.add(br, bl, Gui::Color::black);

	return cage;
}

bool operator!=(const Camera& a, const Camera& b) {
	return a.vertical_fov != b.vertical_fov || a.aspect_ratio != b.aspect_ratio || a.near_plane != b.near_plane
		   || a.aperture_shape != b.aperture_shape || a.aperture_size != b.aperture_size || a.focal_dist != b.focal_dist
	       || a.film.width != b.film.width || a.film.height != b.film.height
	       || a.film.samples != b.film.samples || a.film.max_ray_depth != b.film.max_ray_depth
	       || a.film.sample_pattern != b.film.sample_pattern
	;
}
