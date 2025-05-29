
#include <algorithm>
#include "samplers.h"
#include "../util/rand.h"
#include "../scene/shape.h"


constexpr bool IMPORTANCE_SAMPLING = true;

namespace Samplers {

/**
 *  Return a point selected uniformly at random from the rectangle [0,size.x)x[0,size.y)
*/
Vec2 Rect::sample(RNG &rng) const {
	//A3T1 - step 2 - supersampling

	// Generate two random numbers in [0,1)
	float u = rng.unit();
	float v = rng.unit();
	
	// Scale to the rectangle's dimensions
	return Vec2{u * size.x, v * size.y};
}

// probability density function
float Rect::pdf(Vec2 at) const {
	if (at.x < 0.0f || at.x > size.x || at.y < 0.0f || at.y > size.y) return 0.0f;
	return 1.0f / (size.x * size.y);
}

/**
 * Return a point selected uniformly at random from a circle defined by its center and radius.
*/
Vec2 Circle::sample(RNG &rng) const {
	//A3EC - bokeh - circle sampling
  
	// Uniform sampling in unit circle
	float theta = 2.0f * PI_F * rng.unit();
	float r = std::sqrt(rng.unit()) * radius;
	return center + Vec2(r * std::cos(theta), r * std::sin(theta));
}

/**
 * probability density function
 * Return the pdf of sampling the point 'at' for a circle defined by its center and radius.
 */
float Circle::pdf(Vec2 at) const {
	//A3EC - bokeh - circle pdf

  Vec2 delta = at - center;
	if (delta.norm() > radius) return 0.0f;
	return 1.0f / (PI_F * radius * radius);
}

Vec3 Point::sample(RNG &rng) const {
	return point;
}

float Point::pdf(Vec3 at) const {
	return at == point ? 1.0f : 0.0f;
}

Vec3 Triangle::sample(RNG &rng) const {
	float u = std::sqrt(rng.unit());
	float v = rng.unit();
	float a = u * (1.0f - v);
	float b = u * v;
	return a * v0 + b * v1 + (1.0f - a - b) * v2;
}

float Triangle::pdf(Vec3 at) const {
	float a = 0.5f * cross(v1 - v0, v2 - v0).norm();
	float u = 0.5f * cross(at - v1, at - v2).norm() / a;
	float v = 0.5f * cross(at - v2, at - v0).norm() / a;
	float w = 1.0f - u - v;
	if (u < 0.0f || v < 0.0f || w < 0.0f) return 0.0f;
	if (u > 1.0f || v > 1.0f || w > 1.0f) return 0.0f;
	return 1.0f / a;
}

Vec3 Hemisphere::Uniform::sample(RNG &rng) const {

	float Xi1 = rng.unit();
	float Xi2 = rng.unit();

	float theta = std::acos(Xi1);
	float phi = 2.0f * PI_F * Xi2;

	float xs = std::sin(theta) * std::cos(phi);
	float ys = std::cos(theta);
	float zs = std::sin(theta) * std::sin(phi);

	return Vec3(xs, ys, zs);
}

float Hemisphere::Uniform::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return 1.0f / (2.0f * PI_F);
}

Vec3 Hemisphere::Cosine::sample(RNG &rng) const {

	float phi = rng.unit() * 2.0f * PI_F;
	float cos_theta = std::sqrt(rng.unit());

	float sin_theta = std::sqrt(1 - cos_theta * cos_theta);
	float x = std::cos(phi) * sin_theta;
	float z = std::sin(phi) * sin_theta;
	float y = cos_theta;

	return Vec3(x, y, z);
}

float Hemisphere::Cosine::pdf(Vec3 dir) const {
	if (dir.y < 0.0f) return 0.0f;
	return dir.y / PI_F;
}

// Generate a uniformly random point on the unit sphere.
Vec3 Sphere::Uniform::sample(RNG &rng) const {
	//A3T7 - sphere sampler
   // Generate two uniform random numbers
  float u1 = rng.unit();
  float u2 = rng.unit();
  
  // Proper spherical coordinate convention:
  float phi = 2.0f * PI_F * u1;       // Azimuthal angle [0, 2π]
  float theta = std::acos(1.0f - 2.0f * u2); // Polar angle [0, π]
  
  // Convert to Cartesian coordinates
  float sin_theta = std::sin(theta);
  float x = sin_theta * std::cos(phi);  // x = sinθ cosφ
  float y = sin_theta * std::sin(phi);  // y = sinθ sinφ
  float z = std::cos(theta);            // z = cosθ
  
  return Vec3(x, y, z);
}



float Sphere::Uniform::pdf(Vec3 dir) const {
	return 1.0f / (4.0f * PI_F);
}

Sphere::Image::Image(const HDR_Image& image) {
  //A3T7 - image sampler init
  const auto [_w, _h] = image.dimension();
  w = _w;
  h = _h;
  
  // Allocate space for our PDF and CDF
  _pdf.resize(w * h);
  _cdf.resize(w * h + 1);
  
  // Compute the PDF for each pixel based on flux (radiance * sin(theta))
  total = 0.0f;
  _cdf[0] = 0.0f;
  
  for (uint32_t y = 0; y < h; y++) {
    // Calculate theta for this row - NOTE: y=0 is bottom of image (theta = pi)
    float theta = PI_F * (1.0f - (y + 0.5f) / h);
    float sin_theta = std::sin(theta);
    
    for (uint32_t x = 0; x < w; x++) {
      uint32_t idx = y * w + x;
      
      // Get the pixel color and compute its luminance
      Spectrum pixel_color = image.at(x, y);
      float luminance = pixel_color.luma();
      
      // PDF is proportional to pixel luminance * sin(theta)
      // sin(theta) accounts for the area distortion in the sphere mapping
      _pdf[idx] = luminance * sin_theta;
      total += _pdf[idx];
      
      // Build the CDF as we go
      _cdf[idx + 1] = _cdf[idx] + _pdf[idx];
    }
  }
  
  // Normalize PDF and CDF
  if (total > 0.0f) {
    for (uint32_t i = 0; i < w * h; i++) {
        _pdf[i] /= total;
    }
    for (uint32_t i = 0; i <= w * h; i++) {
        _cdf[i] /= total;
    }
  }
}

Vec3 Sphere::Image::sample(RNG &rng) const {
	if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its sample
    return Uniform{}.sample(rng);
	} else {
		// Step 2: Importance sampling
    // Generate a random number in [0,1)
    float Xi = rng.unit();
    
    // Binary search to find the pixel index from the CDF
    auto it = std::upper_bound(_cdf.begin(), _cdf.end(), Xi);
    uint32_t idx = (uint32_t)std::distance(_cdf.begin(), it) - 1;
    
    // Convert linear index to 2D pixel coordinates
    uint32_t y = idx / w;
    uint32_t x = idx % w;
    
    // Convert to continuous coordinates with small random offset for stratification
    float u = (x + 0.5f) / float(w);  // phi = 2π * u
    float v = (y + 0.5f) / float(h);  // theta = π * (1-v)
    
    // Map to spherical coordinates
    float phi = 2.0f * PI_F * u;
    float theta = PI_F * (1.0f - v);  // Bottom of image is theta = pi
    
    // Convert to Cartesian coordinates
    float sin_theta = std::sin(theta);
    float x_dir = std::cos(phi) * sin_theta;
    float y_dir = std::cos(theta);
    float z_dir = std::sin(phi) * sin_theta;
    
    return Vec3(x_dir, y_dir, z_dir);
    
	}
}

float Sphere::Image::pdf(Vec3 dir) const {
    if(!IMPORTANCE_SAMPLING) {
		// Step 1: Uniform sampling
		// Declare a uniform sampler and return its pdf
    return Uniform{}.pdf(dir);
	} else {
		// A3T7 - image sampler importance sampling pdf

    // Convert direction to image coordinates
    // First, convert to spherical coordinates
    float theta = std::acos(std::clamp(dir.y, -1.0f, 1.0f));
    // We need to handle the special case where dir.x and dir.z are both 0
    float phi = std::atan2(dir.z, dir.x);
    if (phi < 0.0f) phi += 2.0f * PI_F; // Map to [0, 2π)
    
    // Map spherical coordinates to image coordinates
    float u = phi / (2.0f * PI_F);
    float v = 1.0f - (theta / PI_F); // Map [0, π] -> [1, 0]
    
    // Convert to pixel indices
    int x = std::min(static_cast<int>(u * w), (int)w - 1);
    int y = std::min(static_cast<int>(v * h), (int)h - 1);
    int idx = y * w + x;
    
    // Get the PDF value for this pixel
    float pdf_value = _pdf[idx];
    
    // Apply the Jacobian: (w*h)/(2π²*sin(θ))
    // This accounts for the transformation from image space to sphere space
    if (std::sin(theta) > 1e-6f) {
        pdf_value *= (w * h) / (2.0f * PI_F * PI_F * std::sin(theta));
    }
    
    return pdf_value;
	}
}

} // namespace Samplers
