#include "test.h"
#include "rasterizer/pipeline.h"
#include "rasterizer/programs.h"

#include <limits>
#include <iomanip>
#include <algorithm>
#include <unordered_set>

using TestPipeline = Pipeline< PrimitiveType::Lines, Programs::Lambertian, Pipeline_Blend_Replace | Pipeline_Depth_Less | Pipeline_Interp_Flat >;

namespace std {
	template< >
	struct hash< Vec2 > {
		size_t operator()(const Vec2 &v) const {
			static hash< float > hf;
			size_t x = hf(v.x);
			size_t y = hf(v.y);
			return x ^ (y << 16) ^ (y >> (sizeof(y)*8-16));
		}
	};
}

struct Raster {
public:
	uint32_t out_of_raster = 0;
	std::vector< std::string > pixels;
	explicit Raster () {
		pixels.emplace_back(".");
	}
	auto draw(Vec2 const &px, char c) {
		int32_t x = int32_t(std::floor(px.x));
		int32_t y = int32_t(std::floor(px.y));

		if (x < 0 || y < 0 || x > 10 || y > 10) {
			++out_of_raster;
			return;
		}

		if (uint32_t(y) >= pixels.size()) {
			pixels.resize(y+1, "");
		}
		if (uint32_t(x) >= pixels[y].size()) {
			pixels[y].resize(x+1, '.');
		}
		pixels[y][x] = c;
	};

	auto toString() -> std::string {
		std::string info ;
		//square up the raster:
		size_t width = 0;
		for (auto const &row : pixels) {
			width = std::max(width, row.size());
		}
		for (auto &row : pixels) {
			row.resize(width, '.');
		}

		for (uint32_t y = static_cast<uint32_t>(pixels.size()) - 1; y < static_cast<uint32_t>(pixels.size()); --y) {
			info +=  pixels[y] + "\n";
		}
		return info;
	}
};

//check that line produces exactly the listed fragments:
void check_line_covers(std::string const &desc, std::vector< Vec2 > const &line_strip, std::unordered_set< Vec2 > const &expected) {

	std::unordered_set< Vec2 > got;
	for (uint32_t i = 0; i + 1 < line_strip.size(); ++i) {
		TestPipeline::ClippedVertex a,b;
		a.fb_position = Vec3(line_strip[i].x, line_strip[i].y, 0.25f);
		a.inv_w = 1.0f;
		a.attributes.fill(1.0f);
		b.fb_position = Vec3(line_strip[i+1].x, line_strip[i+1].y, 0.75f);
		b.inv_w = 1.0f;
		b.attributes.fill(2.0f);
		TestPipeline::rasterize_line(a, b, [&](TestPipeline::Fragment const &frag){
			got.emplace(frag.fb_position.x, frag.fb_position.y);
		});
	}

	uint32_t matched = 0;
	uint32_t missed = 0;
	uint32_t extra = 0;
	Raster raster, diffRaster, expectRaster;
	for (auto const &f : got) {
		if ((f.x - std::floor(f.x) != 0.5f) || (f.y - std::floor(f.y) != 0.5f)) {
			throw Test::error("Rasterizing '" + desc + "', got fragment at (" + std::to_string(f.x) + ", " + std::to_string(f.y) + "), which isn't at a pixel center.");
		}
		raster.draw(f, '#');
		if (expected.count(f)) {
			diffRaster.draw(f, '#');
			++matched;
		} else {
			diffRaster.draw(f, '!');
			++extra;
		}
	}
	for (auto const &f : expected) {
		expectRaster.draw(f, '#');
		if (!got.count(f)) {
			diffRaster.draw(f, '?');
			++missed;
		}
	}

	if (extra > 0 || missed > 0) {
		//failed!
		std::string info = "Example '" + desc + "' missed " + std::to_string(missed) + " ('?'); had " + std::to_string(extra) + " extra ('!'); and matched " + std::to_string(matched) + " ('#') fragments:\n";
		info += diffRaster.toString();

		if (raster.out_of_raster) info += "(" + std::to_string(raster.out_of_raster) + " out-of-range fragments not plotted.)\n";
		info += "expect: \n" ;
		info +=  expectRaster.toString();
		info += "got: \n" ;
		info +=  raster.toString();

		puts(""); //because "test..."
		info("%s", info.c_str());

		throw Test::error("Example '" + desc + "' didn't match expected.");
	}

	//if nothing extra and nothing missed, success!
	assert(matched == expected.size());
}

//check that line produces exactly the fragments drawn in a fancy picture:
void check_line_covers(std::string const &desc, std::initializer_list< Vec2 > const &line_strip, std::initializer_list< std::string > const &raster_) {
	//convert raster to set of points ( with lower-left being (0,0) ):
	std::vector< std::string > raster(raster_);
	std::unordered_set< Vec2 > expected;
	for (uint32_t y = 0; y < raster.size(); ++y) {
		std::string const &row = raster[raster.size()-1-y];
		for (uint32_t x = 0; x < row.size(); ++x) {
			if (row[x] != '.') {
				expected.emplace(x + 0.5f, y + 0.5f);
			}
		}
	}
	//use list-of-points version:
	check_line_covers(desc, line_strip, expected);
}

//--------------------------------------------------
//entering/exiting diamond at (1,1):
// only lines that *exit* the diamond should produce a fragment.

Test test_a1_task2_diamond_inside("a1.task2.diamond.inside", []() {
	check_line_covers(
		"line inside diamond (1,1)",
		{ Vec2(1.5f, 1.25f), Vec2(1.25f, 1.5f) },
		{"...",
		 "...",
		 "..."}
	);
});


Test test_a1_task2_diamond_outside("a1.task2.diamond.outside", []() {
	check_line_covers(
		"line outside diamond (1,1)",
		{ Vec2(1.125f, 1.25f), Vec2(1.25f, 1.125f) },
		{"...",
		 "...",
		 "..."}
	);
});


//----------------------------
//simple horizontal and vertical lines (set up so that no enter/exit logic needed):

Test test_a1_task2_simple_horizontal("a1.task2.simple.horizontal", []() {
	check_line_covers(
		"horizontal line from (1.125, 1.125) to (4.875, 1.125)",
		{ Vec2(1.125f, 1.125f), Vec2(4.875f, 1.125f) },
		{"......",
		 ".####.",
		 "......"}
	);
});


Test test_a1_task2_simple_vertical("a1.task2.simple.vertical", []() {
	check_line_covers(
		"vertical line from (1.125, 1.125) to (1.125, 4.875)",
		{ Vec2(1.125f, 1.125f), Vec2(1.125f, 4.875f) },
		{"...",
		 ".#.",
		 ".#.",
		 ".#.",
		 ".#.",
		 "..."}
	);
});

Test test_a1_task2_slash_up("a1.task2.slash_up", []() {
	check_line_covers(
		"line from (0.0, 0.5) to (5, 3)",
		{ Vec2(0.0f, 0.5f), Vec2(5.0f, 3.0f) },
		{".....",
		 ".....",
		 "...##",
		 ".##..",
		 "#...."}
	);
});

Test test_a1_task2_slash_up2("a1.task2.slash_up2", []() {
	check_line_covers(
		"line from (0.0, 0.0) to (5, 3)",
		{ Vec2(0.0f, 0.0f), Vec2(5.0f, 3.0f) },
		{".....",
		 ".....",
		 "...##",
		 "..#..",
		 "##..."}
	);
});

Test test_a1_task2_slash_up_steep_singlecolumn("a1.task2.slash.up.steep.singlecolumn", []() {
	check_line_covers(
		"line from (2.5f, 1.2f) to (2.8f, 4.5f)",
		{ Vec2(2.5f, 1.2f), Vec2(2.8f, 4.5f) },
		{".....",
		 "..#..",
		 "..#..",
		 "..#..",
		 "....."}
	);
});

Test test_a1_task2_slash_down("a1.task2.slash.down", []() {
	check_line_covers(
		"line from (0.0, 5.0) to (5.0, 3.0)",
		{ Vec2(0.0f, 5.0f), Vec2(5.0f, 3.0f) },
		{"###..",
		 "...##",
		 ".....",
		 ".....",
		 "....."}
	);
});

Test test_a1_task2_slash_down2("a1.task2.slash.down2", []() {
	check_line_covers(
		"line from (1.0f, 4.0f) to (5.0f, 2.0f)",
		{ Vec2(1.0f, 4.0f), Vec2(5.0f, 2.0f) },
		{".....",
		 ".##..",
		 "...##",
		 ".....",
		 "....."}
	);
});

Test test_a1_task2_slash_down_steep("a1.task2.slash.down.steep", []() {
	check_line_covers(
		"line from (1.0f, 5.0f) to (4.0f, 0.0f)",
		{ Vec2(1.0f, 5.0f), Vec2(4.0f, 0.0f) },
		{".#...",
		 ".#...",
		 "..#..",
		 "...#.",
		 "...#."}
	);
});

Test test_a1_task2_slash_down_steep2("a1.task2.slash.down.steep2", []() {
	check_line_covers(
		"line from (1.0f, 5.0f) to (4.0f, 0.0f)",
		{ Vec2(1.0f, 5.0f), Vec2(4.0f, 0.0f) },
		{".#...",
		 ".#...",
		 "..#..",
		 "...#.",
		 "...#."}
	);
});


Test test_a1_task2_slash_down_steep_singlecolumn("a1.task2.slash.down.steep.singlecolumn", []() {
	check_line_covers(
		"line from (4.5f, 3.8f) to (4.8f, 0.5f)",
		{ Vec2(4.5f, 3.8f), Vec2(4.8f, 0.5f) },
		{".....",
		 "....#",
		 "....#",
		 "....#",
		 "....#"}
	);
});

Test test_a1_task2_diagonal1("a1.task2.diagonal1", []() {
	check_line_covers(
		"line from (0.0, 0.0) to (4.0, 4.0)",
		{ Vec2(0.0f, 0.0f), Vec2(4.0f, 4.0f) },
		{".....",
		 "...#.",
		 "..#..",
		 ".#...",
		 "#...."}
	);
});

Test test_a1_task2_diagona2("a1.task2.diagonal2", []() {
	check_line_covers(
		"line from (0.25, 0.25) to (4.0, 4.0)",
		{ Vec2(0.25f, 0.25f), Vec2(4.0f, 4.0f) },
		{".....",
		 "...#.",
		 "..#..",
		 ".#...",
		 "#...."}
	);
});

Test test_a1_task2_diagona3("a1.task2.diagonal3", []() {
	check_line_covers(
		"line from (0.75, 0.75) to (4.0, 4.0)",
		{ Vec2(0.75f, 0.75f), Vec2(4.0f, 4.0f) },
		{".....",
		 "...#.",
		 "..#..",
		 ".#...",
		 "#...."}
	);
});

Test test_a1_task2_me_diagonal3("a1.task2.me.diagonal3", []() {
	check_line_covers(
		"line from (0.0, 0.75) to (4.25, 5.0)",
		{ Vec2(0.0f, 0.75f), Vec2(4.25f, 5.0f) },
		{"...#.",
		 "..#..",
		 ".#...",
		 "#....",
		 "....."}
	);
});





