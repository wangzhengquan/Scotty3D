
#include "material.h"
#include "../util/rand.h"
#include "../lib/log.h"

namespace Materials {

/**
 * Return direction to incoming light that would be reflected out in direction dir from surface with normal (0,1,0)
*/
Vec3 reflect(Vec3 dir) {
	//A3T5 Materials - reflect helper
  return Vec3(-dir.x, dir.y, -dir.z);
}

/**
 * Use Snell's Law to refract out_dir through the surface.
 * Return the refracted direction. Set was_internal to true if
 * refraction does not occur due to total internal reflection,
 * and false otherwise.

 * The surface normal is (0,1,0)
*/
Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {
	//A3T5 Materials - refract helper
	was_internal = false;
  // ωt is the direction of the outgoing ray
  // Determine entering vs exiting surface
  bool entering = out_dir.y < 0;
  float eta_i = entering ? 1.0f : index_of_refraction;
  float eta_t = entering ? index_of_refraction : 1.0f;
  
  // Compute refraction using Snell's law
  float cos_theta_t = out_dir.y;
  float sin_theta_t = std::sqrt(1 - cos_theta_t * cos_theta_t);
  float sin_theta_i = (eta_t/eta_i) * sin_theta_t;
  
  // Check for total internal reflection
  if (sin_theta_i >= 1.0f) {
      was_internal = true;
      return Vec3();
  }
  
  float cos_theta_i = std::sqrt(1 - sin_theta_i * sin_theta_i);
  
  // Flip cos_theta_i if exiting
  if (!entering) cos_theta_i = -cos_theta_i;
  
  // Compute refracted direction
  return Vec3(
      -(eta_t/eta_i) * out_dir.x,
      cos_theta_i,
      -(eta_t/eta_i) * out_dir.z
  ).unit();
}

/**
 * @return reflectance: fraction of light that is reflected
*/
float schlick(Vec3 in_dir, float index_of_refraction) {
	//A3T5 Materials - Schlick's approximation helper

	// Implement Schlick's approximation of the Fresnel reflection factor.
	float r0 = (1.0f - index_of_refraction) / (1.0f + index_of_refraction);
  r0 = r0 * r0;
  float cos_theta = std::abs(in_dir.y);
  return r0 + (1 - r0) * std::pow(1 - cos_theta, 5);
}

/**
 * Compute the ratio of outgoing/incoming radiance when light from in_dir is reflected through out_dir: (albedo / PI_F) * cos(theta).
 * Note that for Scotty3D, y is the 'up' direction.
*/
Spectrum Lambertian::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	//A3T4: Materials - Lambertian BSDF evaluation

  float cos_theta = in.y;
  if (cos_theta <= 0.0f) return Spectrum{}; // Light coming from below
  
  // Get albedo from texture and divide by PI for energy conservation
  // Lambertian BRDF = albedo * cosθ_i/π 
  return albedo.lock()->evaluate(uv) * (cos_theta / PI_F);
  // return albedo.lock()->evaluate(uv) ;
}

/**
 * Select a scattered light direction at random from the Lambertian BSDF
*/
Scatter Lambertian::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T4: Materials - Lambertian BSDF scattering
	 
  Scatter ret;
  // Sample direction from cosine-weighted hemisphere
  Samplers::Hemisphere::Cosine sampler;
  // sample the direction the light was scatter from from a cosine-weighted hemisphere distribution:
  ret.direction = sampler.sample(rng);
  
  // compute the attenuation of the light using Lambertian::evaluate()
  ret.attenuation = evaluate(out, ret.direction, uv); 
  return ret;
}

/**
 * Compute the PDF for sampling in_dir from the cosine-weighted hemisphere distribution.
*/
float Lambertian::pdf(Vec3 out, Vec3 in) const {
	//A3T4: Materials - Lambertian BSDF probability density function
  
	Samplers::Hemisphere::Cosine sampler; //this might be handy!
  return sampler.pdf(in);
  // if (in.y < 0.0f) return 0.0f;
	// return 1 ;
}

Spectrum Lambertian::emission(Vec2 uv) const {
	return {};
}

std::weak_ptr<Texture> Lambertian::display() const {
	return albedo;
}

void Lambertian::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(albedo);
}

Spectrum Mirror::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
  if (in.y <= 0.0f) return Spectrum{};
	return reflectance.lock()->evaluate(uv);
}

Scatter Mirror::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5: mirror

	// Use reflect to compute the new direction
	// Don't forget that this is a discrete material!
	// Similar to albedo, reflectance represents the ratio of incoming light to reflected light
  Scatter ret;
  ret.direction = reflect(out); // Perfect reflection
  ret.attenuation = evaluate(out, ret.direction, uv);
  return ret;
}

std::weak_ptr<Texture> Mirror::display() const {
	return reflectance;
}

void Mirror::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(reflectance);
}

Spectrum Refract::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Refract::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5 - refract

	// Use refract to determine the new direction - what happens in the total internal reflection case?
    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
	// Don't forget that this is a discrete material!
	// For attenuation, be sure to take a look at the Specular Transimission section of the PBRT textbook for a derivation
	//  You do not need to scale by the Fresnel Coefficient - you'll only need to account for the correct ratio of indices of refraction

  bool was_internal;
  Vec3 refr_dir = refract(out, ior, was_internal);
  
  Scatter ret;
  if (was_internal) {
    // Total internal reflection - treat as mirror
    ret.direction = reflect(out);
    ret.attenuation = Spectrum(1.0f);
  } else {
    ret.direction = refr_dir;
    // Attenuation from transmittance texture with proper scaling
    float eta = out.y > 0 ? 1.0f/ior : ior/1.0f;
    ret.attenuation = transmittance.lock()->evaluate(uv) * (eta*eta);
  }
  return ret;
}

std::weak_ptr<Texture> Refract::display() const {
	return transmittance;
}

void Refract::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(transmittance);
}

Spectrum Glass::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Glass::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	//A3T5 - glass

  // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?
  // What happens upon total internal reflection?
  // When debugging Glass, it may be useful to compare to a pure-refraction BSDF
	// For attenuation, be sure to take a look at the Specular Transimission section of the PBRT textbook for a derivation
	//  You do not need to scale by the Fresnel Coefficient - you'll only need to account for the correct ratio of indices of refraction

  bool was_internal;
  Vec3 refr_dir = refract(out, ior, was_internal);
  
  Scatter ret;
  if (was_internal) {
    // Total internal reflection - must reflect
    ret.direction = reflect(out);
    ret.attenuation = reflectance.lock()->evaluate(uv);
  } else {
    // Choose between reflection and refraction using Fresnel
    // (1) Compute Fresnel coefficient. Tip: Schlick's approximation.
    float fresnel = schlick(out, ior);
    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance
    if (rng.coin_flip(fresnel)) {
      // Reflect case
      ret.direction = reflect(out);
      ret.attenuation = reflectance.lock()->evaluate(uv);
    } else {
      // Refract case
      ret.direction = refr_dir;
      float eta = out.y > 0 ? 1.0f/ior : ior/1.0f;
      ret.attenuation = transmittance.lock()->evaluate(uv) * (eta*eta);
    }
  }
  return ret;
}

std::weak_ptr<Texture> Glass::display() const {
	return transmittance;
}

void Glass::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(reflectance);
	f(transmittance);
}

Spectrum Emissive::evaluate(Vec3 out, Vec3 in, Vec2 uv) const {
	return {};
}

Scatter Emissive::scatter(RNG &rng, Vec3 out, Vec2 uv) const {
	Scatter ret;
	ret.direction = {};
	ret.attenuation = {};
	return ret;
}

std::weak_ptr<Texture> Emissive::display() const {
	return emissive;
}

void Emissive::for_each(const std::function<void(std::weak_ptr<Texture>&)>& f) {
	f(emissive);
}

} // namespace Materials

bool operator!=(const Materials::Lambertian& a, const Materials::Lambertian& b) {
	return a.albedo.lock() != b.albedo.lock();
}

bool operator!=(const Materials::Mirror& a, const Materials::Mirror& b) {
	return a.reflectance.lock() != b.reflectance.lock();
}

bool operator!=(const Materials::Refract& a, const Materials::Refract& b) {
	return a.transmittance.lock() != b.transmittance.lock() || a.ior != b.ior;
}

bool operator!=(const Materials::Glass& a, const Materials::Glass& b) {
	return a.reflectance.lock() != b.reflectance.lock() ||
	       a.transmittance.lock() != b.transmittance.lock() || a.ior != b.ior;
}

bool operator!=(const Materials::Emissive& a, const Materials::Emissive& b) {
	return a.emissive.lock() != b.emissive.lock();
}

bool operator!=(const Material& a, const Material& b) {
	if (a.material.index() != b.material.index()) return false;
	return std::visit(
		[&](const auto& material) {
			return material != std::get<std::decay_t<decltype(material)>>(b.material);
		},
		a.material);
}
