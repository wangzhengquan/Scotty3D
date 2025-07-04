
#include "particles.h"

// Compute the trajectory of this particle for the next dt seconds.
// (1) Build a ray representing the particle's path as if it travelled at constant velocity.
// (2) Intersect the ray with the scene and account for collisions. Be careful when placing
// collision points using the particle radius. Move the particle to its next position.
// (3) Account for acceleration due to gravity after updating position.
// (4) Repeat until the entire time step has been consumed.
// (5) Decrease the particle's age and return 'false' if it should be removed.

// bool Particles::Particle::update(const PT::Aggregate &scene, Vec3 const &gravity, const float radius, const float dt) {
//   //A4T4: particle update

//   float speed = velocity.norm();
//   if (speed == 0.0f) {
//     // Particle is nearly stationary, no need for collision checks.
//     position += velocity * dt;
//     velocity += gravity * dt;
//     age -= dt;
//     return age > 0.0f;
//   }

//   const int steps = 10;
//   float step_time = dt / steps;
//   float time_left = dt;
//   while (time_left > 0) {
//     if (time_left < step_time) {
//       step_time = time_left;
//     }
//     Ray ray(position, velocity, Vec2{0.0f, dt}); 
//     PT::Trace trace = scene.hit(ray);
    
//     if (!trace.hit) {
//       // std::cout << "!hit:" << position << std::endl;
//       position += velocity * step_time;
//       velocity += gravity * step_time;
//       time_left -= step_time;
//       age -= step_time;
     
//     } else {
//       float cos_theta = std::abs(dot(ray.dir, trace.normal));
//       float dist_adj = radius / cos_theta;
//       float collision_dist = trace.distance - dist_adj;
// 			float time_to_collision = collision_dist / speed;
//       // std::cout << "========" << hit.hit <<  ", distance=" << hit.distance << ", dt=" << dt << std::endl;
//       if ( time_to_collision  <= step_time ) {
//         assert(dot(velocity, trace.normal) != 0);
//         // std::cout << "========hit:" << position << std::endl;
//         position = trace.position - dist_adj * ray.dir;
//         velocity = velocity - 2.0f * dot(velocity, trace.normal) * trace.normal;
//         time_left -= time_to_collision;
//         age -= time_to_collision;
//       } else {
//         position += velocity * step_time;
//         velocity += gravity * step_time;
//         time_left -= step_time;
//         age -= step_time;
//       }
//     }
//   }
 
//   return age > 0.0f;
// }

bool Particles::Particle::update(const PT::Aggregate &scene, Vec3 const &gravity, const float radius, const float dt) {
  //A4T4: particle update

  float speed = velocity.norm();
  if (speed == 0.0f) {
    // Particle is nearly stationary, no need for collision checks.
    position += velocity * dt;
    velocity += gravity * dt;
    age -= dt;
    return age > 0.0f;
  }

  float time_left = dt;
  while (time_left > 0) {
     
    Ray ray(position, velocity, Vec2{0.0f, dt * 10});  // , Vec2{0.0f, dt * 10}
    PT::Trace trace = scene.hit(ray);
    
    if (!trace.hit) {
      // std::cout << "!hit:" << position << std::endl;
      position += velocity * time_left;
      velocity += gravity * time_left;
      age -= time_left;
      time_left = 0.0f;
    } else {
      float cos_theta = std::abs(dot(ray.dir, trace.normal));
      float dist_adj = radius / cos_theta;
      float collision_dist = trace.distance - dist_adj;
			float time_to_collision = collision_dist / speed;
      // std::cout << "========" << hit.hit <<  ", distance=" << hit.distance << ", dt=" << dt << std::endl;
      if ( time_to_collision  <= time_left ) {
        assert(dot(velocity, trace.normal) != 0);
        // std::cout << "========hit:" << position << std::endl;
        // position = trace.position - dist_adj * ray.dir;
        position += velocity * time_to_collision;
        velocity = velocity - 2.0f * dot(velocity, trace.normal) * trace.normal;
        age -= time_to_collision;
        time_left -= time_to_collision;
      } else {
        position += velocity * time_left;
        velocity += gravity * time_left;
        age -= time_left;
        time_left = 0.0f;
      }
    }
  }
 
  return age > 0.0f;
}
void Particles::advance(const PT::Aggregate& scene, const Mat4& to_world, float dt) {

	if(step_size < EPS_F) return;

	step_accum += dt;

	while(step_accum > step_size) {
		step(scene, to_world);
		step_accum -= step_size;
	}
}

void Particles::step(const PT::Aggregate& scene, const Mat4& to_world) {

	std::vector<Particle> next;
	next.reserve(particles.size());

	for(Particle& p : particles) {
		if(p.update(scene, gravity, radius, step_size)) {
			next.emplace_back(p);
		}
	}

	if(rate > 0.0f) {

		//helpful when emitting particles:
		float cos = std::cos(Radians(spread_angle) / 2.0f);

		//will emit particle i when i == time * rate
		//(i.e., will emit particle when time * rate hits an integer value.)
		//so need to figure out all integers in [current_step, current_step+1) * step_size * rate
		//compute the range:
		double begin_t = current_step * double(step_size) * double(rate);
		double end_t = (current_step + 1) * double(step_size) * double(rate);

		uint64_t begin_i = uint64_t(std::max(0.0, std::ceil(begin_t)));
		uint64_t end_i = uint64_t(std::max(0.0, std::ceil(end_t)));

		//iterate all integers in [begin, end):
		for (uint64_t i = begin_i; i < end_i; ++i) {
			//spawn particle 'i':

			float y = lerp(cos, 1.0f, rng.unit());
			float t = 2 * PI_F * rng.unit();
			float d = std::sqrt(1.0f - y * y);
			Vec3 dir = initial_velocity * Vec3(d * std::cos(t), y, d * std::sin(t));

			Particle p;
			p.position = to_world * Vec3(0.0f, 0.0f, 0.0f);
			p.velocity = to_world.rotate(dir);
			p.age = lifetime; //NOTE: could adjust lifetime based on index
			next.push_back(p);
		}
	}

	particles = std::move(next);
	current_step += 1;
}

void Particles::reset() {
	particles.clear();
	step_accum = 0.0f;
	current_step = 0;
	rng.seed(seed);
}

bool operator!=(const Particles& a, const Particles& b) {
	return a.gravity != b.gravity
	|| a.radius != b.radius
	|| a.initial_velocity != b.initial_velocity
	|| a.spread_angle != b.spread_angle
	|| a.lifetime != b.lifetime
	|| a.rate != b.rate
	|| a.step_size != b.step_size
	|| a.seed != b.seed;
}
