
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"
#include <iostream>
#include <stack>

namespace PT {

struct BVHBuildData {
	BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst) {
	}
	size_t start; ///< start index into the primitive array
	size_t range; ///< range of index into the primitive array
	size_t node;  ///< address to update
};

struct SAHBucketData {
	BBox bb;          ///< bbox of all primitives
	size_t num_prims; ///< number of primitives in the bucket
};

// template<typename Primitive>
// void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
// 	//A3T3 - build a bvh

// 	// Keep these
// 	nodes.clear();
// 	primitives = std::move(prims);
// 	// Construct a BVH from the given vector of primitives and maximum leaf
// 	// size configuration.
 
// }
// In src/pathtracer/bvh.cpp
template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	std::cout << "max_leaf_size:" << max_leaf_size << std::endl;
    nodes.clear();
    primitives = std::move(prims);
    if (primitives.empty()) return;
    root_idx = build_node(primitives.begin(), primitives.end(), max_leaf_size);
}

template<typename Primitive>
size_t BVH<Primitive>::build_node(typename std::vector<Primitive>::iterator first, typename std::vector<Primitive>::iterator last, size_t max_leaf_size)
{
	size_t start = first - primitives.begin();
	size_t size = last - first;
	// std::cout << "start: " << start << ", size: " << size << ", total: " << primitives.size()<< std::endl;
	BBox bb;
	for (auto it = first; it < last; ++it){
		bb.enclose(it->bbox());
	}
	// std::cout << "build_node:" << bb << std::endl;
	if (size <= max_leaf_size) {
		// Create leaf node
		return new_node(bb, start, size);
	}

	// Choose split axis (longest axis)
	Vec3 extent = bb.max - bb.min;
	int axis = extent.x > extent.y ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2);
  Vec3 pivot = bb.center();
	auto middle = std::partition(first, last, [&](const Primitive& em) {
		return em.bbox().center()[axis] < pivot[axis]; 
	});
	 
	if (middle != first) {
		size_t l = build_node(first, middle, max_leaf_size);
		size_t r = build_node(middle, last, max_leaf_size);
		return new_node(bb, start, size, l, r);
	} else {
		std::cout << "size:" << size << std::endl;
		return new_node(bb, start, size);
	}
	
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
	//A3T3 - traverse your BVH
	// Implement ray - BVH intersection test. A ray intersects
	// with a BVH aggregate if and only if it intersects a primitive in
	// the BVH that is not an aggregate.

	// The starter code simply iterates through all the primitives.
	// Again, remember you can use hit() on any Primitive value.

	//TODO: replace this code with a more efficient traversal:
	// Trace ret;
	// for(const Primitive& prim : primitives) {
	//     Trace hit = prim.hit(ray);
	//     ret = Trace::min(ret, hit);
	// }
	// return ret;

	Trace ret;
	if (nodes.empty()) return ret;
	
	std::function<void(const Node& node)> find_closest_hit = [&](const Node& node) {
		if (node.is_leaf()) {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				Trace hit = primitives[i].hit(ray);
		// std::cout << "2 hit:" << hit.hit << std::endl;
				ret = Trace::min(ret, hit);
			}
		} else {
			Vec2 t_l(0.f, FLT_MAX), t_r(0.f, FLT_MAX);
			const Node& l = nodes[node.l];
			const Node& r = nodes[node.r];
			// find_closest_hit(l);
			// find_closest_hit(r);
			bool hit_l = l.bbox.hit(ray, t_l);
			bool hit_r = r.bbox.hit(ray, t_r);
			
			if (hit_l && hit_r) {
				const Node *first, *second;
				float t1, t2;
				if (t_l.x <= t_r.x ) {
					first = &l;
					second = &r;
					t1 = t_l.x;
					t2 = t_r.x;
				} else {
					first = &r;
					second = &l;
					t1 = t_r.x;
					t2 = t_l.x;
				}
				find_closest_hit(*first);
				if(!ret.hit || t2 < ret.distance) {
					find_closest_hit(*second);
				}
				
			} else if(hit_l ){
				find_closest_hit(l);

			} else if (hit_r) {
				find_closest_hit(r);
			}
			
		}
	};

	find_closest_hit(nodes[root_idx]);
	 
	return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
	build(std::move(prims), max_leaf_size);
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
	nodes.clear();
	return std::move(primitives);
}

template<typename Primitive>
template<typename P>
typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const {
	BVH<Primitive> ret;
	ret.nodes = nodes;
	ret.primitives = primitives;
	ret.root_idx = root_idx;
	return ret;
}

template<typename Primitive> Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const {
	if (primitives.empty()) return {};
	int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
	return primitives[n].sample(rng, from);
}

template<typename Primitive>
float BVH<Primitive>::pdf(Ray ray, const Mat4& T, const Mat4& iT) const {
	if (primitives.empty()) return 0.0f;
	float ret = 0.0f;
	for (auto& prim : primitives) ret += prim.pdf(ray, T, iT);
	return ret / primitives.size();
}

template<typename Primitive> void BVH<Primitive>::clear() {
	nodes.clear();
	primitives.clear();
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {
	// A node is a leaf if l == r, since all interior nodes must have distinct children
	return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
	Node n;
	n.bbox = box;
	n.start = start;
	n.size = size;
	n.l = l;
	n.r = r;
	nodes.push_back(n);
	return nodes.size() - 1;
}
 
template<typename Primitive> BBox BVH<Primitive>::bbox() const {
	if(nodes.empty()) return BBox{Vec3{0.0f}, Vec3{0.0f}};
	return nodes[root_idx].bbox;
}

template<typename Primitive> size_t BVH<Primitive>::n_primitives() const {
	return primitives.size();
}

template<typename Primitive>
uint32_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, uint32_t level,
                                   const Mat4& trans) const {

	std::stack<std::pair<size_t, uint32_t>> tstack;
	tstack.push({root_idx, 0u});
	uint32_t max_level = 0u;

	if (nodes.empty()) return max_level;

	while (!tstack.empty()) {

		auto [idx, lvl] = tstack.top();
		max_level = std::max(max_level, lvl);
		const Node& node = nodes[idx];
		tstack.pop();

		Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
		GL::Lines& add = lvl == level ? active : lines;

		BBox box = node.bbox;
		box.transform(trans);
		Vec3 min = box.min, max = box.max;

		auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

		edge(min, Vec3{max.x, min.y, min.z});
		edge(min, Vec3{min.x, max.y, min.z});
		edge(min, Vec3{min.x, min.y, max.z});
		edge(max, Vec3{min.x, max.y, max.z});
		edge(max, Vec3{max.x, min.y, max.z});
		edge(max, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
		edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
		edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

		if (!node.is_leaf()) {
			tstack.push({node.l, lvl + 1});
			tstack.push({node.r, lvl + 1});
		} else {
			for (size_t i = node.start; i < node.start + node.size; i++) {
				uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
				max_level = std::max(c + lvl, max_level);
			}
		}
	}
	return max_level;
}

template class BVH<Triangle>;
template class BVH<Instance>;
template class BVH<Aggregate>;
template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
