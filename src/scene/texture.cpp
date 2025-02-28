
#include "texture.h"

#include <iostream>

namespace Textures {


Spectrum sample_nearest(HDR_Image const &image, Vec2 uv) {
	//clamp texture coordinates, convert to [0,w]x[0,h] pixel space:
	float x = image.w * std::clamp(uv.x, 0.0f, 1.0f);
	float y = image.h * std::clamp(uv.y, 0.0f, 1.0f);

	//the pixel with the nearest center is the pixel that contains (x,y):
	int32_t ix = int32_t(std::floor(x));
	int32_t iy = int32_t(std::floor(y));

	//texture coordinates of (1,1) map to (w,h), and need to be reduced:
	ix = std::min(ix, int32_t(image.w) - 1);
	iy = std::min(iy, int32_t(image.h) - 1);

	return image.at(ix, iy);
}

Spectrum sample_bilinear(HDR_Image const &image, Vec2 uv) {
	// A1T6: sample_bilinear
	//TODO: implement bilinear sampling strategy on texture 'image'
	// std::cout << "\nimage: (" << image.w << "," << image.h << "), uv:" << uv << std::endl;
	float x = image.w * uv.x;
	float y = image.h * uv.y;

	// Find the four nearest texels
	int32_t x0 = std::clamp(static_cast<int32_t>(std::floor(x - 0.5)), 0, static_cast<int32_t>(image.w) - 1);
	int32_t y0 = std::clamp(static_cast<int32_t>(std::floor(y - 0.5)), 0, static_cast<int32_t>(image.h) - 1);
	int32_t x1 = std::clamp(x0 + 1, 0, static_cast<int32_t>(image.w) - 1);
	int32_t y1 = std::clamp(y0 + 1, 0, static_cast<int32_t>(image.h) - 1);

	// Compute fractional parts
	float s = x - x0 - 0.5;
	float t = y - y0 - 0.5;

	// Sample the four nearest texels
	Spectrum texel00 = image.at(x0, y0), texel10 = image.at(x1, y0),
	  			 texel01 = image.at(x0, y1), texel11 = image.at(x1, y1);

	// Perform bilinear interpolation
	Spectrum bottom = texel00 * (1.0f - s) + texel10 * s;
	Spectrum top = texel01 * (1.0f - s) + texel11 * s;
	Spectrum result = bottom * (1.0f - t) + top * t;

	return result;
}

Spectrum sample_trilinear(HDR_Image const &base, std::vector< HDR_Image > const &levels, Vec2 uv, float lod) {
	// A1T6: sample_trilinear
	//TODO: implement trilinear sampling strategy on using mip-map 'levels'
	int32_t level0 = static_cast<int32_t>(std::floor(lod));
	int32_t level1 = level0 + 1;

	// Clamp level0 and level1 to valid range
	level0 = std::clamp(level0, 0, static_cast<int32_t>(levels.size()) );
	level1 = std::clamp(level1, 0, static_cast<int32_t>(levels.size()) );

	// Perform bilinear sampling on both levels
	Spectrum sample0, sample1;
	if (level0 == 0) {
			// Use the base image for level0
			sample0 = sample_bilinear(base, uv);
	} else {
			// Use the corresponding mipmap level for level0
			sample0 = sample_bilinear(levels[level0 - 1], uv);
	}

	if (level1 == 0) {
			// Use the base image for level1
			sample1 = sample_bilinear(base, uv);
	} else {
			// Use the corresponding mipmap level for level1
			sample1 = sample_bilinear(levels[level1 - 1], uv);
	}

	// Compute the fractional part of the LOD
	float frac = lod - level0;
	// Linearly interpolate between the two samples
	Spectrum result = sample0 * (1.0f - frac) + sample1 * frac;
	return result;
}

/*
 * generate_mipmap- generate mipmap levels from a base image.
 *  base: the base image
 *  levels: pointer to vector of levels to fill (must not be null)
 *
 * generates a stack of levels [1,n] of sizes w_i, h_i, where:
 *   w_i = max(1, floor(w_{i-1})/2)
 *   h_i = max(1, floor(h_{i-1})/2)
 *  with:
 *   w_0 = base.w
 *   h_0 = base.h
 *  and n is the smalles n such that w_n = h_n = 1
 *
 * each level should be calculated by downsampling a blurred version
 * of the previous level to remove high-frequency detail.
 *
 */
void generate_mipmap(HDR_Image const &base, std::vector< HDR_Image > *levels_) {
	assert(levels_);
	auto &levels = *levels_;


	{ // allocate sublevels sufficient to scale base image all the way to 1x1:
		int32_t num_levels = static_cast<int32_t>(std::log2(std::max(base.w, base.h)));
		assert(num_levels >= 0);

		levels.clear();
		levels.reserve(num_levels);

		uint32_t width = base.w;
		uint32_t height = base.h;
		for (int32_t i = 0; i < num_levels; ++i) {
			assert(!(width == 1 && height == 1)); //would have stopped before this if num_levels was computed correctly

			width = std::max(1u, width / 2u);
			height = std::max(1u, height / 2u);

			levels.emplace_back(width, height);
		}
		assert(width == 1 && height == 1);
		assert(levels.size() == uint32_t(num_levels));
	}

	//now fill in the levels using a helper:
	//downsample:
	// fill in dst to represent the low-frequency component of src
	auto downsample = [](HDR_Image const &src, HDR_Image &dst) {
		//dst is half the size of src in each dimension:
		assert(std::max(1u, src.w / 2u) == dst.w);
		assert(std::max(1u, src.h / 2u) == dst.h);

		// A1T6: generate
		//TODO: Write code to fill the levels of the mipmap hierarchy by downsampling

		//Be aware that the alignment of the samples in dst and src will be different depending on whether the image is even or odd.
		//dst is half the size of src in each dimension:
    // Iterate over each pixel in the destination level
    for (uint32_t y = 0; y < dst.h; ++y) {
        for (uint32_t x = 0; x < dst.w; ++x) {
            // Compute the corresponding 2x2 block in the source level
            uint32_t src_x = x * 2;
            uint32_t src_y = y * 2;

            // Initialize the sum of the 2x2 block
            Spectrum sum = Spectrum(0.0f, 0.0f, 0.0f);
            uint32_t count = 0;

            // Iterate over the 2x2 block in the source level
            for (uint32_t dy = 0; dy < 2; ++dy) {
                for (uint32_t dx = 0; dx < 2; ++dx) {
                    // Check if the source pixel is within bounds
                    if (src_x + dx < src.w && src_y + dy < src.h) {
                        sum += src.at(src_x + dx, src_y + dy);
                        count++;
                    }
                }
            }

            // Compute the average color of the 2x2 block
            if (count > 0) {
                dst.at(x, y) = sum / float(count);
            } else {
                dst.at(x, y) = Spectrum(0.0f, 0.0f, 0.0f); // Fallback for empty blocks
            }
        }
    }
	};

	std::cout << "Regenerating mipmap (" << levels.size() << " levels): [" << base.w << "x" << base.h << "]";
	std::cout.flush();
	for (uint32_t i = 0; i < levels.size(); ++i) {
		HDR_Image const &src = (i == 0 ? base : levels[i-1]);
		HDR_Image &dst = levels[i];
		std::cout << " -> [" << dst.w << "x" << dst.h << "]"; std::cout.flush();

		downsample(src, dst);
	}
	std::cout << std::endl;
	
}

Image::Image(Sampler sampler_, HDR_Image const &image_) {
	sampler = sampler_;
	image = image_.copy();
	update_mipmap();
}

Spectrum Image::evaluate(Vec2 uv, float lod) const {
	if (image.w == 0 && image.h == 0) return Spectrum();
	if (sampler == Sampler::nearest) {
		return sample_nearest(image, uv);
	} else if (sampler == Sampler::bilinear) {
		return sample_bilinear(image, uv);
	} else {
		return sample_trilinear(image, levels, uv, lod);
	}
}

void Image::update_mipmap() {
	if (sampler == Sampler::trilinear) {
		generate_mipmap(image, &levels);
	} else {
		levels.clear();
	}
}

GL::Tex2D Image::to_gl() const {
	return image.to_gl(1.0f);
}

void Image::make_valid() {
	update_mipmap();
}

Spectrum Constant::evaluate(Vec2 uv, float lod) const {
	return color * scale;
}

} // namespace Textures
bool operator!=(const Textures::Constant& a, const Textures::Constant& b) {
	return a.color != b.color || a.scale != b.scale;
}

bool operator!=(const Textures::Image& a, const Textures::Image& b) {
	return a.image != b.image;
}

bool operator!=(const Texture& a, const Texture& b) {
	if (a.texture.index() != b.texture.index()) return false;
	return std::visit(
		[&](const auto& data) { return data != std::get<std::decay_t<decltype(data)>>(b.texture); },
		a.texture);
}
