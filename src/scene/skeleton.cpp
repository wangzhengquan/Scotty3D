#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

/**
 * what axes (in local bone space) Bone::pose should rotate around.
*/
void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const {
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	//y axis points in the direction of extent:
	y = extent.unit();
	//if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid()) {
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	//x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x,y) * y).unit();
	if (!x.valid()) {
		//if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x,y) * y).unit(); //(this should do nothing)
	}

	//z computed from x,y:
	z = cross(x,y);

	//x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector< Mat4 > Skeleton::bind_pose() const {
	//A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)
	std::vector< Mat4 > bind;
	bind.reserve(bones.size());

	//NOTE: bones is guaranteed to be ordered such that parents appear before child bones.
	for (auto const &bone : bones) {
		Mat4 bone_transform;
    if (bone.parent == -1U) {
      // Root bone: translate by skeleton base
      bone_transform = Mat4::translate(base);
    } else {
      // Child bone: parent transform * translation by parent's extent
      bone_transform = bind[bone.parent] * Mat4::translate(bones[bone.parent].extent);
    }
    
    bind.emplace_back(bone_transform);
	}

	assert(bind.size() == bones.size()); //should have a transform for every bone.
	return bind;

  
}

std::vector< Mat4 > Skeleton::current_pose() const {
  //A4T2a: bone-to-skeleton transformations in the current pose

	//Similar to bind_pose(), but takes rotation from Bone::pose into account.
	// (and translation from Skeleton::base_offset!)

	//Useful functions:
	//Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
	//Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.

	std::vector< Mat4 > current;
  current.reserve(bones.size());

  for (auto const &bone : bones) {
    Mat4 bone_transform;
    
    // Compute rotation axes for this bone
    Vec3 x_axis, y_axis, z_axis;
    bone.compute_rotation_axes(&x_axis, &y_axis, &z_axis);
    // Create rotation matrices for each axis (in order: x, y, z)
    Mat4 rot_x = Mat4::angle_axis(bone.pose.x, x_axis);
    Mat4 rot_y = Mat4::angle_axis(bone.pose.y, y_axis);
    Mat4 rot_z = Mat4::angle_axis(bone.pose.z, z_axis);
    
    // Combined rotation: Rz * Ry * Rx
    Mat4 rotation = rot_z * rot_y * rot_x;
    if (bone.parent == -1U) {
        // Root bone: translate by base + base_offset, then apply rotation
        Mat4 translation = Mat4::translate(base + base_offset);
        bone_transform = translation * rotation;
    } else {
        // Child bone: parent transform * translation by parent's extent * rotation
        Mat4 translation = Mat4::translate(bones[bone.parent].extent);
        bone_transform = current[bone.parent] * translation * rotation;
    }
    
    current.emplace_back(bone_transform);
  }

  assert(current.size() == bones.size());
  return current;

}

/**
 * Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.
*/
std::vector< Vec3 > Skeleton::gradient_in_current_pose() const {
  //A4T2b: IK gradient

	//The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector< Vec3 > gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});
  auto pose_matrices = current_pose();
	//loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
  for (const auto& handle : handles) {
    if (!handle.enabled) continue;

    BoneIndex ik_bone_idx = handle.bone;
    Vec3 target_pos = handle.target;

    const auto& ik_bone = bones[ik_bone_idx];
    // pos_i(q) = T_i(q) * extent_i
    Vec3 tip_pos = (pose_matrices[ik_bone_idx] * Vec4(ik_bone.extent, 1.0f)).xyz();
    Vec3 error_vec = tip_pos - target_pos;

    BoneIndex current_bone_idx = ik_bone_idx;
    while (current_bone_idx != -1U) {
      const auto& bone = bones[current_bone_idx];
      Mat4 T = pose_matrices[current_bone_idx];
      Vec3 center = (T * Vec4(0.0f, 0.0f, 0.0f, 1.0f)).xyz();

      Vec3 x, y, z;
      bone.compute_rotation_axes(&x, &y, &z);

      Vec3 axis_x = (T * Vec4(1.0f, 0.0f, 0.0f, 0.0f)).xyz();
      Vec3 axis_y = (T * Mat4::angle_axis(-bone.pose.x, x) * Vec4(0.0f, 1.0f, 0.0f, 0.0f)).xyz();
      Vec3 axis_z =  (T * Mat4::angle_axis(-bone.pose.x, x) * Mat4::angle_axis(-bone.pose.y, y) * Vec4(0.0f, 0.0f, 1.0f, 0.0f)).xyz();

      Vec3 partial_pos_x = cross(axis_x, tip_pos - center);
      Vec3 partial_pos_y = cross(axis_y, tip_pos - center);
      Vec3 partial_pos_z = cross(axis_z, tip_pos - center);
   
      gradient[current_bone_idx] += Vec3(dot(error_vec, partial_pos_x), dot(error_vec, partial_pos_y), dot(error_vec, partial_pos_z));
  //  std::cout << "partial_pos_x=" << partial_pos_x << ", "<< "partial_pos_y=" << partial_pos_y << ", " << "partial_pos_z=" << partial_pos_z << std::endl;
  //  std::cout << "gradient[current_bone_idx]=" << gradient[current_bone_idx] << std::endl;
      current_bone_idx = bone.parent;
    }
  }
  return gradient;
}
 

bool Skeleton::solve_ik(uint32_t steps) {
	//A4T2b - gradient descent
	//check which handles are enabled
  bool any_enabled = false;
	for (const auto& handle : handles) {
		if (handle.enabled) {
			any_enabled = true;
			break;
		}
	}
	if (!any_enabled) return true;
	
	// A small fixed step size for gradient descent.
	float tau = 1.0f;
  //run `steps` iterations
	for (uint32_t i = 0; i < steps; ++i) {
    //call gradient_in_current_pose() to compute d loss / d pose
		std::vector<Vec3> grad = gradient_in_current_pose();
		float grad_norm_sq = 0.0f;
		for (const auto& g : grad) {
			grad_norm_sq += g.norm_squared();
		}
		// Check for convergence
    // if at a local minimum (e.g., gradient is near-zero), return 'true'.
		if (grad_norm_sq < 1e-8f) {
			return true;
		}
		// Apply gradient descent step
		for (size_t j = 0; j < bones.size(); ++j) {
			bones[j].pose -= tau * grad[j];
		}
	}
  //if run through all steps, return `false`.
	return false;
}


Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p) {
	//A4T3: bone weight computation (closest point helper)

    // Return the closest point to 'p' on the line segment from a to b

	//Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)

    return Vec3{};
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const {
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; //avoid complaints about unused mesh

	//A4T3: bone weight computation

	//visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)

	//be sure to use bone positions in the bind pose (not the current pose!)

	//you should fill in the helper closest_point_on_line_segment() before working on this function

}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector< Mat4 > const &bind, std::vector< Mat4 > const &current) {
	assert(bind.size() == current.size());


	//A4T3: linear blend skinning

	//one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	// then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map< Halfedge_Mesh::VertexCRef, Vec3 > skinned_positions;
	std::unordered_map< Halfedge_Mesh::HalfedgeCRef, Vec3 > skinned_normals;
	//reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi) {
		skinned_positions.emplace(vi, vi->position); //PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
		//NOTE: vertices with empty bone_weights should remain in place.

		//circulate corners at this vertex:
		auto h = vi->halfedge;
		do {
			//NOTE: could skip if h->face->boundary, since such corners don't get emitted

			skinned_normals.emplace(h, h->corner_normal); //PLACEHOLDER! Replace with code that properly transforms the normal vector! Make sure that you normalize correctly.

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	//Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	Indexed_Mesh result = Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges); //PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.

	return result;
}

void Skeleton::for_bones(const std::function<void(Bone&)>& f) {
	for (auto& bone : bones) {
		f(bone);
	}
}


void Skeleton::erase_bone(BoneIndex bone) {
	assert(bone < bones.size());
	//update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (bones[b].parent == -1U) continue;
		if (bones[b].parent == bone) {
			assert(b > bone); //topological sort!
			//keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		} else if (bones[b].parent > bone) {
			assert(b > bones[b].parent); //topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	//update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */) {
		if (handles[h].bone == bone) {
			erase_handle(h);
		} else if (handles[h].bone > bone) {
			handles[h].bone -= 1;
			++h;
		} else {
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle) {
	assert(handle < handles.size());

	//nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}


Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent) {
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &b : bones) {
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id)) ++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	//all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target) {
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	//all other parameters left as default.

	//slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set< uint32_t > used;
	for (auto const &h : handles) {
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id)) ++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}


Skeleton Skeleton::copy() {
	//turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid() {
	for (uint32_t b = 0; b < bones.size(); ++b) {
		if (!(bones[b].parent == -1U || bones[b].parent < b)) {
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty()) {
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h) {
		if (handles[h].bone >= HandleIndex(bones.size())) {
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const {
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const {
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy() {
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
