
#include "../geometry/spline.h"

/**
 * Given a time, find the nearest positions & tangent values
 * defined by the control point map.
*/
template<typename T> T Spline<T>::at(float time) const {
	// A4T1b: Evaluate a Catumull-Rom spline

  // Handle edge cases first
  if (knots.empty()) {
      return T(); // Default value for type T
  }
  
  if (knots.size() == 1) {
      return knots.begin()->second; // Return the single knot value
  }
  
  // If time is before first knot, return first knot value
  if (time <= knots.begin()->first) {
      return knots.begin()->second;
  }
  
  // If time is after last knot, return last knot value
  if (time >= knots.rbegin()->first) {
      return knots.rbegin()->second;
  }
  
  // Find the interval containing the query time
  // k2 points to first knot with time > query time
  auto k2_iter = knots.upper_bound(time);
  auto k1_iter = std::prev(k2_iter); // k1 is the knot before k2
  
  // Get the times and values for k1 and k2
  float t1 = k1_iter->first;
  float t2 = k2_iter->first;
  T p1 = k1_iter->second;
  T p2 = k2_iter->second;
  
  // Now we need to find k0 and k3 (or create virtual ones)
  float t0, t3;
  T p0, p3;
  
  // Find k0 (knot before k1)
  if (k1_iter == knots.begin()) {
      // No knot before k1, create virtual knot by mirroring
      t0 = t1 - (t2 - t1);
      p0 = p1 - (p2 - p1);
  } else {
      auto k0_iter = std::prev(k1_iter);
      t0 = k0_iter->first;
      p0 = k0_iter->second;
  }
  
  // Find k3 (knot after k2)
  auto k3_iter = std::next(k2_iter);
  if (k3_iter == knots.end()) {
      // No knot after k2, create virtual knot by mirroring
      t3 = t2 + (t2 - t1);
      p3 = p2 + (p2 - p1);
  } else {
      t3 = k3_iter->first;
      p3 = k3_iter->second;
  }
  
  // Calculate tangents using Catmull-Rom formula
  T m0 = (p2 - p0) / (t2 - t0);
  T m1 = (p3 - p1) / (t3 - t1);
  
  // Normalize time to [0,1] for the interval [t1, t2]
  float normalized_time = (time - t1) / (t2 - t1);
  
  // Scale tangents by the interval length for proper interpolation
  // This is necessary because cubic_unit_spline expects tangents 
  // scaled for a unit interval
  float interval_length = t2 - t1;
  T scaled_m0 = m0 * interval_length;
  T scaled_m1 = m1 * interval_length;
  
  // Use cubic_unit_spline to interpolate
  return cubic_unit_spline(normalized_time, p1, p2, scaled_m0, scaled_m1);
}

/**
 * Given time in [0,1] compute the cubic spline coefficients and use them to compute
 * the interpolated value at time 'time' based on the positions & tangents
*/
template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {
	// A4T1a: Hermite Curve over the unit interval
	
  // Compute powers of time for efficiency and readability
  float t = time;
  float t2 = t * t;
  float t3 = t2 * t;
  
  // Compute the four Hermite basis functions
  float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
  float h10 = t3 - 2.0f * t2 + t;
  float h01 = -2.0f * t3 + 3.0f * t2;
  float h11 = t3 - t2;
  
  // Combine using Hermite interpolation formula:
  // p(t) = h00(t)*p0 + h10(t)*m0 + h01(t)*p1 + h11(t)*m1
  return h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;
}

template class Spline<float>;
template class Spline<double>;
template class Spline<Vec4>;
template class Spline<Vec3>;
template class Spline<Vec2>;
template class Spline<Mat4>;
template class Spline<Spectrum>;
