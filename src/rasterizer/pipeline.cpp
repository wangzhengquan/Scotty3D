// clang-format off
#include "pipeline.h"

#include <iostream>

#include "../lib/log.h"
#include "../lib/mathlib.h"
#include "framebuffer.h"
#include "sample_pattern.h"

/**
 * compute the distance of p to the line  determined by two points v0 and v1。
 */
inline static float dist_of_p_to_l (Vec2 const& v0, Vec2 const& v1, Vec2 const& p)  {
		return (v0.y -  v1.y) * p.x + ( v1.x - v0.x) * p.y  + v0.x *  v1.y -  v1.x * v0.y;
};

inline static float triangle_area(Vec2 const& v0, Vec2 const& v1, Vec2 const& v2) {
		return (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x);
};


template<PrimitiveType primitive_type, class Program, uint32_t flags>
void Pipeline<primitive_type, Program, flags>::run(std::vector<Vertex> const& vertices,
                                                   typename Program::Parameters const& parameters,
                                                   Framebuffer* framebuffer_) {
	// Framebuffer must be non-null:
	assert(framebuffer_);
	auto& framebuffer = *framebuffer_;

	// A1T7: sample loop
	// TODO: update this function to rasterize to *all* sample locations in the framebuffer.
	//  	 This will probably involve inserting a loop of the form:
	// 		 	std::vector< Vec3 > const &samples = framebuffer.sample_pattern.centers_and_weights;
	//      	for (uint32_t s = 0; s < samples.size(); ++s) { ... }
	//   	 around some subset of the code.
	// 		 You will also need to transform the input and output of the rasterize_* functions to
	// 	     account for the fact they deal with pixels centered at (0.5,0.5).
	
	// Get the sample pattern and number of samples per pixel
	std::vector<Vec3> const& samples = framebuffer.sample_pattern.centers_and_weights;
	uint32_t samples_per_pixel = static_cast<uint32_t>(samples.size());

	std::vector<ShadedVertex> shaded_vertices;
	shaded_vertices.reserve(vertices.size());

	//--------------------------
	// shade vertices:
	for (auto const& v : vertices) {
		ShadedVertex sv;
		Program::shade_vertex(parameters, v.attributes, &sv.clip_position, &sv.attributes);
		shaded_vertices.emplace_back(sv);
	}

	//--------------------------
	// assemble + clip + homogeneous divide vertices:
	std::vector<ClippedVertex> clipped_vertices;

	// reserve some space to avoid reallocations later:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		// clipping lines can never produce more than one vertex per input vertex:
		clipped_vertices.reserve(shaded_vertices.size());
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		// clipping triangles can produce up to 8 vertices per input vertex:
		clipped_vertices.reserve(shaded_vertices.size() * 8);
	}
	// clang-format off

	//coefficients to map from clip coordinates to framebuffer (i.e., "viewport") coordinates:
	//x: [-1,1] -> [0,width]
	//y: [-1,1] -> [0,height]
	//z: [-1,1] -> [0,1] (OpenGL-style depth range)
	Vec3 const clip_to_fb_scale = Vec3{
		framebuffer.width / 2.0f,
		framebuffer.height / 2.0f,
		0.5f
	};
	Vec3 const clip_to_fb_offset = Vec3{
		0.5f * framebuffer.width,
		0.5f * framebuffer.height,
		0.5f
	};

	// helper used to put output of clipping functions into clipped_vertices:
	auto emit_vertex = [&](ShadedVertex const& sv) {
		ClippedVertex cv;
		float inv_w = 1.0f / sv.clip_position.w;
		cv.fb_position = clip_to_fb_scale * inv_w * sv.clip_position.xyz() + clip_to_fb_offset;
		cv.inv_w = inv_w;
		cv.attributes = sv.attributes;
		clipped_vertices.emplace_back(cv);
	};

	// actually do clipping:
	if constexpr (primitive_type == PrimitiveType::Lines) {
		for (uint32_t i = 0; i + 1 < shaded_vertices.size(); i += 2) {
			clip_line(shaded_vertices[i], shaded_vertices[i + 1], emit_vertex);
		}
	} else if constexpr (primitive_type == PrimitiveType::Triangles) {
		for (uint32_t i = 0; i + 2 < shaded_vertices.size(); i += 3) {
			clip_triangle(shaded_vertices[i], shaded_vertices[i + 1], shaded_vertices[i + 2], emit_vertex);
		}
	} else {
		static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
	}

	//--------------------------
	// rasterize primitives:
	for (uint32_t s = 0; s < samples_per_pixel; ++s) {
		// Shift the positions of the ClippedVertices to account for the sample offset
		Vec2 sample_offset = Vec2(samples[s].x - 0.5f, samples[s].y - 0.5f);

		// Shift the positions of the ClippedVertices
		std::vector<ClippedVertex> supersample_clipped_vertices;
		supersample_clipped_vertices.reserve(clipped_vertices.size());
		for (auto const& cv : clipped_vertices) {
				ClippedVertex shifted_cv = cv;
				shifted_cv.fb_position.x += sample_offset.x;
				shifted_cv.fb_position.y += sample_offset.y;
				supersample_clipped_vertices.emplace_back(shifted_cv);
		}

		std::vector<Fragment> fragments;

		// helper used to put output of rasterization functions into fragments:
		auto emit_fragment = [&](Fragment const& f) { fragments.emplace_back(f); };

		// actually do rasterization:
		if constexpr (primitive_type == PrimitiveType::Lines) {
			for (uint32_t i = 0; i + 1 < supersample_clipped_vertices.size(); i += 2) {
				rasterize_line(supersample_clipped_vertices[i], supersample_clipped_vertices[i + 1], emit_fragment);
			}
		} else if constexpr (primitive_type == PrimitiveType::Triangles) {
			for (uint32_t i = 0; i + 2 < supersample_clipped_vertices.size(); i += 3) {
				rasterize_triangle(supersample_clipped_vertices[i], supersample_clipped_vertices[i + 1], supersample_clipped_vertices[i + 2], emit_fragment);
			}
		} else {
			static_assert(primitive_type == PrimitiveType::Lines, "Unsupported primitive type.");
		}

		//--------------------------
		// depth test + shade + blend fragments:
		uint32_t out_of_range = 0; // check if rasterization produced fragments outside framebuffer 
									// (indicates something is wrong with clipping)
		for (auto const& f : fragments) {

			// fragment location (in pixels):
			int32_t x = (int32_t)std::floor(f.fb_position.x);
			int32_t y = (int32_t)std::floor(f.fb_position.y);

			// if clipping is working properly, this condition shouldn't be needed;
			// however, it prevents crashes while you are working on your clipping functions,
			// so we suggest leaving it in place:
			if (x < 0 || (uint32_t)x >= framebuffer.width || 
					y < 0 || (uint32_t)y >= framebuffer.height) {
				++out_of_range;
				continue;
			}

			// local names that refer to destination sample in framebuffer:
			float& fb_depth = framebuffer.depth_at(x, y, s);
			Spectrum& fb_color = framebuffer.color_at(x, y, s);


			// depth test:
			if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Always) {
				// "Always" means the depth test always passes.
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Never) {
				// "Never" means the depth test never passes.
				continue; //discard this fragment
			} else if constexpr ((flags & PipelineMask_Depth) == Pipeline_Depth_Less) {
				// "Less" means the depth test passes when the new fragment has depth less than the stored depth.
				// A1T4: Depth_Less
				// TODO: implement depth test! We want to only emit fragments that have a depth less than the stored depth, hence "Depth_Less".
				if (f.fb_position.z >= fb_depth){
					continue;
				}
			} else {
				static_assert((flags & PipelineMask_Depth) <= Pipeline_Depth_Always, "Unknown depth test flag.");
			}

			// if depth test passes, and depth writes aren't disabled, write depth to depth buffer:
			if constexpr (!(flags & Pipeline_DepthWriteDisableBit)) {
				fb_depth = f.fb_position.z;
			}

			// shade fragment:
			ShadedFragment sf;
			sf.fb_position = f.fb_position;
			Program::shade_fragment(parameters, f.attributes, f.derivatives, &sf.color, &sf.opacity);

			// write color to framebuffer if color writes aren't disabled:
			if constexpr (!(flags & Pipeline_ColorWriteDisableBit)) {
				// blend fragment:
				if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Replace) {
					fb_color = sf.color;
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Add) {
					// A1T4: Blend_Add
					// TODO: framebuffer color should have fragment color multiplied by fragment opacity added to it.
					fb_color += sf.color * sf.opacity;
				} else if constexpr ((flags & PipelineMask_Blend) == Pipeline_Blend_Over) {
					// A1T4: Blend_Over
					// TODO: set framebuffer color to the result of "over" blending (also called "alpha blending") the fragment color over the framebuffer color, using the fragment's opacity
					// 		 You may assume that the framebuffer color has its alpha premultiplied already, and you just want to compute the resulting composite color
					fb_color = sf.color * sf.opacity + fb_color * (1.0f - sf.opacity);
				} else {
					static_assert((flags & PipelineMask_Blend) <= Pipeline_Blend_Over, "Unknown blending flag.");
				}
			}
		}
		if (out_of_range > 0) {
			if constexpr (primitive_type == PrimitiveType::Lines) {
				warn("Produced %d fragments outside framebuffer; this indicates something is likely "
						"wrong with the clip_line function.",
						out_of_range);
			} else if constexpr (primitive_type == PrimitiveType::Triangles) {
				warn("Produced %d fragments outside framebuffer; this indicates something is likely "
						"wrong with the clip_triangle function.",
						out_of_range);
			}
		}
	}
}

// -------------------------------------------------------------------------
// clipping functions

// helper to interpolate between vertices:
template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ShadedVertex const& a, ShadedVertex const& b, float t) -> ShadedVertex {
	ShadedVertex ret;
	ret.clip_position = (b.clip_position - a.clip_position) * t + a.clip_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}


/*
 * clip_line - clip line to portion with -w <= x,y,z <= w, emit vertices of clipped line (if non-empty)
 *  	va, vb: endpoints of line
 *  	emit_vertex: call to produce truncated line
 *
 * If clipping shortens the line, attributes of the shortened line should respect the pipeline's interpolation mode.
 * 
 * If no portion of the line remains after clipping, emit_vertex will not be called.
 *
 * The clipped line should have the same direction as the full line.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_line(ShadedVertex const& va, ShadedVertex const& vb,
                                      std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// Determine portion of line over which:
	// 		pt = (b-a) * t + a
	//  	-pt.w <= pt.x <= pt.w
	//  	-pt.w <= pt.y <= pt.w
	//  	-pt.w <= pt.z <= pt.w
	// ... as a range [min_t, max_t]:

	float min_t = 0.0f;
	float max_t = 1.0f;

	// want to set range of t for a bunch of equations like:
	//    a.x + t * ba.x <= a.w + t * ba.w
	// so here's a helper:
	auto clip_range = [&min_t, &max_t](float l, float dl, float r, float dr) {
		// restrict range such that:
		// l + t * dl <= r + t * dr
		// re-arranging:
		//  l - r <= t * (dr - dl)
		if (dr == dl) {
			// want: l - r <= 0
			if (l - r > 0.0f) {
				// works for none of range, so make range empty:
				min_t = 1.0f;
				max_t = 0.0f;
			}
		} else if (dr > dl) {
			// since dr - dl is positive:
			// want: (l - r) / (dr - dl) <= t
			min_t = std::max(min_t, (l - r) / (dr - dl));
		} else { // dr < dl
			// since dr - dl is negative:
			// want: (l - r) / (dr - dl) >= t
			max_t = std::min(max_t, (l - r) / (dr - dl));
		}
	};

	// local names for clip positions and their difference:
	Vec4 const& a = va.clip_position;
	Vec4 const& b = vb.clip_position;
	Vec4 const ba = b - a;

	// -a.w - t * ba.w <= a.x + t * ba.x <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.x, ba.x);
	clip_range(a.x, ba.x, a.w, ba.w);
	// -a.w - t * ba.w <= a.y + t * ba.y <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.y, ba.y);
	clip_range(a.y, ba.y, a.w, ba.w);
	// -a.w - t * ba.w <= a.z + t * ba.z <= a.w + t * ba.w
	clip_range(-a.w, -ba.w, a.z, ba.z);
	clip_range(a.z, ba.z, a.w, ba.w);

	if (min_t < max_t) {
		if (min_t == 0.0f) {
			emit_vertex(va);
		} else {
			ShadedVertex out = lerp(va, vb, min_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
		if (max_t == 1.0f) {
			emit_vertex(vb);
		} else {
			ShadedVertex out = lerp(va, vb, max_t);
			// don't interpolate attributes if in flat shading mode:
			if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
				out.attributes = va.attributes;
			}
			emit_vertex(out);
		}
	}
}

/*
 * clip_triangle - clip triangle to portion with -w <= x,y,z <= w, emit resulting shape as triangles (if non-empty)
 *  	va, vb, vc: vertices of triangle
 *  	emit_vertex: call to produce clipped triangles (three calls per triangle)
 *
 * If clipping truncates the triangle, attributes of the new vertices should respect the pipeline's interpolation mode.
 * 
 * If no portion of the triangle remains after clipping, emit_vertex will not be called.
 *
 * The clipped triangle(s) should have the same winding order as the full triangle.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::clip_triangle(
	ShadedVertex const& va, ShadedVertex const& vb, ShadedVertex const& vc,
	std::function<void(ShadedVertex const&)> const& emit_vertex) {
	// A1EC: clip_triangle
	// TODO: correct code!
	emit_vertex(va);
	emit_vertex(vb);
	emit_vertex(vc);
}

// -------------------------------------------------------------------------
// rasterization functions

template<PrimitiveType p, class P, uint32_t F>
auto Pipeline<p, P, F>::lerp(ClippedVertex const& a, ClippedVertex const& b, float t) -> ClippedVertex {
	ClippedVertex ret;
	ret.fb_position = (b.fb_position - a.fb_position) * t + a.fb_position;
	for (uint32_t i = 0; i < ret.attributes.size(); ++i) {
		ret.attributes[i] = (b.attributes[i] - a.attributes[i]) * t + a.attributes[i];
	}
	return ret;
}
/*
 * rasterize_line:
 * calls emit_fragment( frag ) for every pixel "covered" by the line (va.fb_position.xy, vb.fb_position.xy).
 *
 *    a pixel (x,y) is "covered" by the line if it exits the inscribed diamond:
 * 
 *        (x+0.5,y+1)
 *        /        \
 *    (x,y+0.5)  (x+1,y+0.5)
 *        \        /
 *         (x+0.5,y)
 *
 *    to avoid ambiguity, we consider diamonds to contain their left and bottom points
 *    but not their top and right points. 
 * 
 * 	  since 45 degree lines breaks this rule, our rule in general is to rasterize the line as if its
 *    endpoints va and vb were at va + (e, e^2) and vb + (e, e^2) where no smaller nonzero e produces 
 *    a different rasterization result. 
 *    We will not explicitly check for 45 degree lines along the diamond edges (this will be extra credit),
 *    but you should be able to handle 45 degree lines in every other case (such as starting from pixel centers)
 *
 * for each such diamond, pass Fragment frag to emit_fragment, with:
 *  - frag.fb_position.xy set to the center (x+0.5,y+0.5)
 *  - frag.fb_position.z interpolated linearly between va.fb_position.z and vb.fb_position.z
 *  - frag.attributes set to va.attributes (line will only be used in Interp_Flat mode)
 *  - frag.derivatives set to all (0,0)
 *
 * when interpolating the depth (z) for the fragments, you may use any depth the line takes within the pixel
 * (i.e., you don't need to interpolate to, say, the closest point to the pixel center)
 *
 * If you wish to work in fixed point, check framebuffer.h for useful information about the framebuffer's dimensions.
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line(
    ClippedVertex const& va, ClippedVertex const& vb,
    std::function<void(Fragment const&)> const& emit_fragment) {
		return rasterize_line1(va, vb, emit_fragment);
}

template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line1(
    ClippedVertex const& va, ClippedVertex const& vb,
    std::function<void(Fragment const&)> const& emit_fragment) {
    if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
        assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
    }

    // Extract positions
    Vec2 a = va.fb_position.xy();
    Vec2 b = vb.fb_position.xy();

    // Determine the direction of the line
    Vec2 delta = b - a;
    Vec2 abs_delta = delta.abs();

    // Determine the major axis (x or y)
    bool flip_by_xy = abs_delta.y > abs_delta.x;
    if (flip_by_xy) {
        std::swap(a.x, a.y);
        std::swap(b.x, b.y);
        std::swap(delta.x, delta.y);
        std::swap(abs_delta.x, abs_delta.y);
    }

    // // Ensure we are always going from left to right
    if (a.x > b.x) {
        std::swap(a, b);
        delta = b - a;
    }

		int32_t y_step =  delta.y == 0 ? 0 : delta.y > 0 ? 1 : -1;
		 
		auto f_xy = [&](float x, float y) -> float {
			return (a.y - b.y) * x + (b.x - a.x) * y  + a.x * b.y - b.x * a.y;
		};
    
		float x = std::floor(a.x) + 0.5f;
		float y = std::floor(a.y) + 0.5f;
		float d = f_xy(x, y + y_step * 0.5);
		while (x < b.x) {
			if (d * y_step <= 0) {
				y += y_step;
				d += (a.y - b.y) + y_step * (b.x - a.x) ;
			} else {
				d += (a.y - b.y);
			}
			// std::cout << "====== "<< x << "," << y << std::endl;
			// Linear interpolation for z
			float t = (x - a.x) / (b.x - a.x);
			float z = va.fb_position.z + t * (vb.fb_position.z - va.fb_position.z);

			// Emit fragment
			Fragment frag;
			frag.fb_position = flip_by_xy ? Vec3(y, x, z) : Vec3(x, y, z);
			frag.attributes = va.attributes ;
			// for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
			// 	frag.attributes[i] = (vb.attributes[i] - va.attributes[i]) * t + va.attributes[i];
			// }
			frag.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(frag);

			x++;
		}
}

template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line2(
    ClippedVertex const& va, ClippedVertex const& vb,
    std::function<void(Fragment const&)> const& emit_fragment) {
    if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
        assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
    }

    // Extract positions
    Vec2 a = va.fb_position.xy();
    Vec2 b = vb.fb_position.xy();

    // Determine the direction of the line
    Vec2 delta = b - a;
    Vec2 abs_delta = delta.abs();

    // Determine the major axis (x or y)
    bool flip_by_xy = abs_delta.y > abs_delta.x;
    if (flip_by_xy) {
        std::swap(a.x, a.y);
        std::swap(b.x, b.y);
        std::swap(delta.x, delta.y);
        std::swap(abs_delta.x, abs_delta.y);
    }

    // // Ensure we are always going from left to right
    if (a.x > b.x) {
        std::swap(a, b);
        delta = b - a;
    }

		// int32_t y_step =  delta.y == 0 ? 0 : delta.y > 0 ? 1 : -1;
		int32_t flip_by_x = delta.y < 0 ? -1 : 1;
		a.y *= flip_by_x;
		b.y *= flip_by_x;
		auto f_xy = [&](float x, float y) -> float {
			return (a.y - b.y) * x + (b.x - a.x) * y  + a.x * b.y - b.x * a.y;
		};
    
		float x = std::floor(a.x) + 0.5f;
		float y = std::floor(a.y) + 0.5f;
		float d = f_xy(x, y + 0.5);
		 
		while (x < b.x) {
			if (d <= 0) {
				y += 1;
				d += (a.y - b.y) + (b.x - a.x);
			} else {
				d += (a.y - b.y);
			}
			// std::cout << "====== "<< x << "," << y << std::endl;
			// Linear interpolation for z
			float t = (x - a.x) / (b.x - a.x);
			float z = va.fb_position.z + t * (vb.fb_position.z - va.fb_position.z);

			// Emit fragment
			Fragment frag;
			frag.fb_position = flip_by_xy ? Vec3(y * flip_by_x, x, z) : Vec3(x, y * flip_by_x, z);
			frag.attributes = va.attributes ;
			// for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
			// 	frag.attributes[i] = (vb.attributes[i] - va.attributes[i]) * t + va.attributes[i];
			// }
			frag.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(frag);
			x++;
		}
}

// Bresenham Line Algorithm
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_line3(
    ClippedVertex const& va, ClippedVertex const& vb,
    std::function<void(Fragment const&)> const& emit_fragment) {
    if constexpr ((flags & PipelineMask_Interp) != Pipeline_Interp_Flat) {
        assert(0 && "rasterize_line should only be invoked in flat interpolation mode.");
    }

    // Extract positions
    Vec2 a = va.fb_position.xy();
    Vec2 b = vb.fb_position.xy();

    // Determine the direction of the line
    Vec2 delta = b - a;
    Vec2 abs_delta = delta.abs();

    // Determine the major axis (x or y)
    bool flip_by_xy = abs_delta.y > abs_delta.x;
    if (flip_by_xy) {
        std::swap(a.x, a.y);
        std::swap(b.x, b.y);
        std::swap(delta.x, delta.y);
        std::swap(abs_delta.x, abs_delta.y);
    }

    // // Ensure we are always going from left to right
    if (a.x > b.x) {
        std::swap(a, b);
        delta = b - a;
    }

		int32_t flip_by_x = delta.y < 0 ? -1 : 1;
		a.y *= flip_by_x;
		b.y *= flip_by_x;
		 
		// std::cout << "======" << delta << std::endl;
		// std::cout << "=====" << (b.x - a.x) << "," << (b.y - a.y) << std::endl;
		float m =  (b.y - a.y) / (b.x - a.x);
		float x = std::floor(a.x) + 0.5, y = (x - a.x) * m + a.y;

		while (x < b.x) {
			// Linear interpolation for z
			float t = (x - a.x) / (b.x - a.x);
			float z = va.fb_position.z + t * (vb.fb_position.z - va.fb_position.z);

			// Emit fragment
			Fragment frag;
			frag.fb_position = flip_by_xy ? Vec3((std::floor(y) + 0.5) * flip_by_x, x, z) : Vec3(x, (std::floor(y) + 0.5) * flip_by_x , z);
			frag.attributes = va.attributes ;
			// for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
			// 	frag.attributes[i] = (vb.attributes[i] - va.attributes[i]) * t + va.attributes[i];
			// }
			frag.derivatives.fill(Vec2(0.0f, 0.0f));
			emit_fragment(frag);

			x++;
			y += m;
		}
}

/*
 * rasterize_triangle(a,b,c,emit) calls 'emit(frag)' at every location
 *  	(x+0.5,y+0.5) (where x,y are integers) covered by triangle (a,b,c).
 *
 * The emitted fragment should have:
 * - frag.fb_position.xy = (x+0.5, y+0.5)
 * - frag.fb_position.z = linearly interpolated fb_position.z from a,b,c (NOTE: does not depend on Interp mode!)
 * - frag.attributes = depends on Interp_* flag in flags:
 *   - if Interp_Flat: copy from va.attributes
 *   - if Interp_Smooth: interpolate as if (a,b,c) is a 2D triangle flat on the screen
 *   - if Interp_Correct: use perspective-correct interpolation
 * - frag.derivatives = derivatives w.r.t. fb_position.x and fb_position.y of the first frag.derivatives.size() attributes.
 *
 * Notes on derivatives:
 * 	The derivatives are partial derivatives w.r.t. screen locations. That is:
 *    derivatives[i].x = d/d(fb_position.x) attributes[i]
 *    derivatives[i].y = d/d(fb_position.y) attributes[i]
 *  You may compute these derivatives analytically or numerically.
 *
 *  See section 8.12.1 "Derivative Functions" of the GLSL 4.20 specification for some inspiration. (*HOWEVER*, the spec is solving a harder problem, and also nothing in the spec is binding on your implementation)
 *
 *  One approach is to rasterize blocks of four fragments and use forward and backward differences to compute derivatives.
 *  To assist you in this approach, keep in mind that the framebuffer size is *guaranteed* to be even. (see framebuffer.h)
 *
 * Notes on coverage:
 *  If two triangles are on opposite sides of the same edge, and a
 *  fragment center lies on that edge, rasterize_triangle should
 *  make sure that exactly one of the triangles emits that fragment.
 *  (Otherwise, speckles or cracks can appear in the final render.)
 * 
 *  For degenerate (co-linear) triangles, you may consider them to not be on any side of an edge.
 * 	Thus, even if two degnerate triangles share an edge that contains a fragment center, you don't need to emit it.
 *  You will not lose points for doing something reasonable when handling this case
 *
 *  This is pretty tricky to get exactly right!
 *
 */
template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
		return rasterize_triangle1(va, vb, vc, emit_fragment);
}

template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle2(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
	// Extract positions
	Vec2 a = va.fb_position.xy();
	Vec2 b = vb.fb_position.xy();
	Vec2 c = vc.fb_position.xy();

	// Bounding box of the triangle
	float min_x = std::min({a.x, b.x, c.x});
	float max_x = std::max({a.x, b.x, c.x});
	float min_y = std::min({a.y, b.y, c.y});
	float max_y = std::max({a.y, b.y, c.y});

	// Area functions
	// float area = triangle_area(a, b, c);
	float fa = dist_of_p_to_l(b, c, a),
	  		fb = dist_of_p_to_l(c, a, b),
	  		fc = dist_of_p_to_l(a, b, c);
 
	Vec2 off_screen_point(-1, -1);
	bool sa = fa * dist_of_p_to_l(b, c, off_screen_point) > 0, 
			 sb = fb * dist_of_p_to_l(c, a, off_screen_point) > 0, 
			 sc = fc * dist_of_p_to_l(a, b, off_screen_point) > 0;
  float xa = (b.y - c.y) / fa, xb = (c.y - a.y) / fb, xc = (a.y - b.y) / fc,
				ya = (c.x - b.x) / fa, yb = (a.x - c.x) / fb, yc = (b.x - a.x) / fc;
	// Rasterize the triangle
	for (float y = std::floor(min_y) + 0.5; y <= max_y; ++y) {
		float x = std::floor(min_x) + 0.5;
		float alpha = dist_of_p_to_l(b, c, Vec2 (x , y)) / fa;
		float beta = dist_of_p_to_l(c, a, Vec2 (x , y)) / fb;
		float gamma = dist_of_p_to_l(a, b, Vec2 (x , y)) / fc;
		for (; x <= max_x; ++x, alpha += xa, beta += xb, gamma += xc) {
// std::cout << "=======" << alpha << "," << beta << "," << gamma << "," << triangle_area(a, b, c) << std::endl;
			// if (alpha >= 0 && beta >= 0 && gamma >= 0) {
			if ((alpha > 0 || (alpha ==0 && sa)) && (beta > 0 || (beta==0 && sb)) && (gamma > 0 || (gamma==0 && sc) )) {
				// Interpolate depth
				float z = alpha * va.fb_position.z + beta * vb.fb_position.z + gamma * vc.fb_position.z;
				// Interpolate attributes based on interpolation mode
				Fragment frag;
				frag.fb_position = Vec3(x, y, z);
// std::cout << "find=======" << frag.fb_position << std::endl;
				if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
					// A1T3: flat triangles
						frag.attributes = va.attributes;
				} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
					// A1T5: screen-space smooth triangles
					// Interpolate attributes linearly using barycentric coordinates
					for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
							frag.attributes[i] = alpha * va.attributes[i] + beta * vb.attributes[i] + gamma * vc.attributes[i];
							// printf("attributes[%d]: %f, %f, %f \n", i, alpha * va.attributes[i], beta * vb.attributes[i], gamma * vc.attributes[i]);
					}
					
					// Compute derivatives using forward differences
					for (uint32_t i = 0; i < frag.derivatives.size(); ++i) {
							// Compute d/dx: difference between current fragment and next fragment in x direction
							float dx = (alpha + xa) * va.attributes[i] + (beta + xb) * vb.attributes[i] + (gamma + xc) * vc.attributes[i] - frag.attributes[i];
							// Compute d/dy: difference between current fragment and next fragment in y direction 
							float dy = (alpha + ya) * va.attributes[i] + (beta + yb) * vb.attributes[i] + (gamma + yc) * vc.attributes[i] - frag.attributes[i];
							// printf("dx attributes[%d]: %f, %f, %f \n", i, (alpha + 1.0f) * va.attributes[i], beta * vb.attributes[i], gamma * vc.attributes[i]);
							// printf("dy attributes[%d]: %f, %f, %f \n", i, alpha * va.attributes[i], (beta + 1.0f) * vb.attributes[i], gamma * vc.attributes[i]);
							frag.derivatives[i] = Vec2(dx, dy);
							// std::cout << "derivatives[" << i << "]" << frag.derivatives[i] << std::endl;
					}
					
				} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
						// A1T5: perspective-correct triangles
						// Interpolate 1/w and attributes/w using barycentric coordinates
						float inv_w = alpha * va.inv_w + beta * vb.inv_w + gamma * vc.inv_w;
						for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
								float attr_over_w = alpha * va.attributes[i] * va.inv_w + beta * vb.attributes[i] * vb.inv_w + gamma * vc.attributes[i] * vc.inv_w;
								frag.attributes[i] = attr_over_w / inv_w;
						}
						// Compute derivatives using forward differences
						for (uint32_t i = 0; i < frag.derivatives.size(); ++i) {
								float dx = ((alpha + xa) * va.attributes[i] * va.inv_w + (beta + xb) * vb.attributes[i] * vb.inv_w + (gamma + xc) * vc.attributes[i] * vc.inv_w) / inv_w  - frag.attributes[i];
								float dy = ((alpha + ya) * va.attributes[i] * va.inv_w + (beta + yb) * vb.attributes[i] * vb.inv_w + (gamma + yc) * vc.attributes[i] * vc.inv_w) / inv_w  - frag.attributes[i];
								frag.derivatives[i] = Vec2(dx, dy);
								// std::cout << "derivatives[" << i << "]" << frag.derivatives[i] << std::endl;
						}
						 
				}

				emit_fragment(frag);
			}
		}
	}
}


template<PrimitiveType p, class P, uint32_t flags>
void Pipeline<p, P, flags>::rasterize_triangle1(
	ClippedVertex const& va, ClippedVertex const& vb, ClippedVertex const& vc,
	std::function<void(Fragment const&)> const& emit_fragment) {
	// Extract positions
	Vec2 a = va.fb_position.xy();
	Vec2 b = vb.fb_position.xy();
	Vec2 c = vc.fb_position.xy();

	// Bounding box of the triangle
	float min_x = std::min({a.x, b.x, c.x});
	float max_x = std::max({a.x, b.x, c.x});
	float min_y = std::min({a.y, b.y, c.y});
	float max_y = std::max({a.y, b.y, c.y});

	// Area functions
	// float area = triangle_area(a, b, c);
	float fa = dist_of_p_to_l(b, c, a);
	float fb = dist_of_p_to_l(c, a, b);
	float fc = dist_of_p_to_l(a, b, c);
 
	Vec2 off_screen_point(-1, -1);
	bool sa = fa * dist_of_p_to_l(b, c, off_screen_point) > 0, 
			 sb = fb * dist_of_p_to_l(c, a, off_screen_point) > 0, 
			 sc = fc * dist_of_p_to_l(a, b, off_screen_point) > 0;

	// Rasterize the triangle
	for (float y = std::floor(min_y) + 0.5; y <= max_y; ++y) {
		for (float x = std::floor(min_x) + 0.5; x <= max_x; ++x) {
			Vec2 v(x , y);
			float alpha = dist_of_p_to_l(b, c, v) / fa;
			float beta = dist_of_p_to_l(c, a, v) / fb;
			float gamma = dist_of_p_to_l(a, b, v) / fc;
// std::cout << "=======" << alpha << "," << beta << "," << gamma << "," << triangle_area(a, b, c) << std::endl;
			// if (alpha >= 0 && beta >= 0 && gamma >= 0) {
			if ((alpha > 0 || (alpha ==0 && sa)) && (beta > 0 || (beta==0 && sb)) && (gamma > 0 || (gamma==0 && sc) )) {
				// Interpolate depth
				float z = alpha * va.fb_position.z + beta * vb.fb_position.z + gamma * vc.fb_position.z;
				// Interpolate attributes based on interpolation mode
				Fragment frag;
				frag.fb_position = Vec3(v, z);
// std::cout << "find=======" << frag.fb_position << std::endl;
				if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Flat) {
					// A1T3: flat triangles
						frag.attributes = va.attributes;
				} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Smooth) {
					// A1T5: screen-space smooth triangles
					// Interpolate attributes linearly using barycentric coordinates
					for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
							frag.attributes[i] = alpha * va.attributes[i] + beta * vb.attributes[i] + gamma * vc.attributes[i];
							//std::cout << "attributes[]" << 
							// printf("attributes[%d]: %f, %f, %f \n", i, alpha * va.attributes[i], beta * vb.attributes[i], gamma * vc.attributes[i]);
					}
					
					// Compute derivatives using forward differences
					for (uint32_t i = 0; i < frag.derivatives.size(); ++i) {
						 
							float alpha2 = dist_of_p_to_l(b, c, Vec2(x+1 , y)) / fa;
							float beta2 = dist_of_p_to_l(c, a, Vec2(x+1 , y)) / fb;
							float gamma2 = dist_of_p_to_l(a, b, Vec2(x+1 , y)) / fc;
							float dx = alpha2 * va.attributes[i] + beta2 * vb.attributes[i] + gamma2 * vc.attributes[i] - frag.attributes[i];
							// printf("dx2=%f\n", dx2);
							alpha2 = dist_of_p_to_l(b, c, Vec2(x , y+1)) / fa;
							beta2 = dist_of_p_to_l(c, a, Vec2(x , y+1)) / fb;
							gamma2 = dist_of_p_to_l(a, b, Vec2(x , y+1)) / fc;
							float dy = alpha2 * va.attributes[i] + beta2 * vb.attributes[i] + gamma2 * vc.attributes[i] - frag.attributes[i];
							 
							// printf("dy attributes[%d]: %f, %f, %f \n", i, alpha * va.attributes[i], (beta + 1.0f) * vb.attributes[i], gamma * vc.attributes[i]);
							frag.derivatives[i] = Vec2(dx, dy);
							// std::cout << "derivatives[" << i << "]" << frag.derivatives[i] << std::endl;
						
					}
					
				} else if constexpr ((flags & PipelineMask_Interp) == Pipeline_Interp_Correct) {
						// A1T5: perspective-correct triangles
						// Interpolate 1/w and attributes/w using barycentric coordinates
						float inv_w = alpha * va.inv_w + beta * vb.inv_w + gamma * vc.inv_w;
						for (uint32_t i = 0; i < frag.attributes.size(); ++i) {
								float attr_over_w = alpha * va.attributes[i] * va.inv_w + beta * vb.attributes[i] * vb.inv_w + gamma * vc.attributes[i] * vc.inv_w;
								frag.attributes[i] = attr_over_w / inv_w;
						}
						// Compute derivatives using forward differences
						for (uint32_t i = 0; i < frag.derivatives.size(); ++i) {
							float alpha2 = dist_of_p_to_l(b, c, Vec2(x+1 , y)) / fa;
							float beta2 = dist_of_p_to_l(c, a, Vec2(x+1 , y)) / fb;
							float gamma2 = dist_of_p_to_l(a, b, Vec2(x+1 , y)) / fc;
							float dx = (alpha2 * va.attributes[i] * va.inv_w  + beta2 * vb.attributes[i] * vb.inv_w  + gamma2 * vc.attributes[i] * vc.inv_w) / inv_w - frag.attributes[i];
							// printf("dx2=%f\n", dx2);
							alpha2 = dist_of_p_to_l(b, c, Vec2(x , y+1)) / fa;
							beta2 = dist_of_p_to_l(c, a, Vec2(x , y+1)) / fb;
							gamma2 = dist_of_p_to_l(a, b, Vec2(x , y+1)) / fc;
							float dy = (alpha2 * va.attributes[i] * va.inv_w  + beta2 * vb.attributes[i] * vb.inv_w  + gamma2 * vc.attributes[i] * vc.inv_w) / inv_w - frag.attributes[i];
							frag.derivatives[i] = Vec2(dx, dy); 
							 
						}
				}

				emit_fragment(frag);
			}
		}
	}
}
 
//-------------------------------------------------------------------------
// compile instantiations for all programs and blending and testing types:

#include "programs.h"

template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Smooth>;
template struct Pipeline<PrimitiveType::Triangles, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Correct>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Add | Pipeline_Depth_Less | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Always | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Never | Pipeline_Interp_Flat>;
template struct Pipeline<PrimitiveType::Lines, Programs::Lambertian,
                         Pipeline_Blend_Over | Pipeline_Depth_Less | Pipeline_Interp_Flat>;