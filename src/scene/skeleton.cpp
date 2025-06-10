#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

/**
 * what axes (in local bone space) Bone::pose should rotate around.
 */
void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const
{
  assert(x_ && y_ && z_);
  auto &x = *x_;
  auto &y = *y_;
  auto &z = *z_;

  // y axis points in the direction of extent:
  y = extent.unit();
  // if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
  if (!y.valid())
  {
    y = Vec3{0.0f, 1.0f, 0.0f};
  }

  // x gets skeleton's 'x' axis projected to be orthogonal to 'y':
  x = Vec3{1.0f, 0.0f, 0.0f};
  x = (x - dot(x, y) * y).unit();
  if (!x.valid())
  {
    // if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
    x = Vec3{0.0f, 0.0f, 1.0f};
    x = (x - dot(x, y) * y).unit(); //(this should do nothing)
  }

  // z computed from x,y:
  z = cross(x, y);

  // x,z rotated by roll:
  float cr = std::cos(roll / 180.0f * PI_F);
  float sr = std::sin(roll / 180.0f * PI_F);
  // x = cr * x + sr * -z;
  // z = cross(x,y);
  std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector<Mat4> Skeleton::bind_pose() const
{
  // A4T2a: bone-to-skeleton transformations in the bind pose
  //(the bind pose does not rotate by Bone::pose)
  std::vector<Mat4> bind;
  bind.reserve(bones.size());

  // NOTE: bones is guaranteed to be ordered such that parents appear before child bones.
  for (auto const &bone : bones)
  {
    Mat4 bone_transform;
    if (bone.parent == -1U)
    {
      // Root bone: translate by skeleton base
      bone_transform = Mat4::translate(base);
    }
    else
    {
      // Child bone: parent transform * translation by parent's extent
      bone_transform = bind[bone.parent] * Mat4::translate(bones[bone.parent].extent);
    }

    bind.emplace_back(bone_transform);
  }

  assert(bind.size() == bones.size()); // should have a transform for every bone.
  return bind;
}

std::vector<Mat4> Skeleton::current_pose() const
{
  // A4T2a: bone-to-skeleton transformations in the current pose

  // Similar to bind_pose(), but takes rotation from Bone::pose into account.
  //  (and translation from Skeleton::base_offset!)

  // Useful functions:
  // Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
  // Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.

  std::vector<Mat4> current;
  current.reserve(bones.size());

  for (auto const &bone : bones)
  {
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
    if (bone.parent == -1U)
    {
      // Root bone: translate by base + base_offset, then apply rotation
      Mat4 translation = Mat4::translate(base + base_offset);
      bone_transform = translation * rotation;
    }
    else
    {
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
std::pair<std::vector<Vec3>, std::vector<Mat4>> Skeleton::gradient_in_current_pose() const
{
  // A4T2b: IK gradient

  // The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
  std::vector<Vec3> gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});
  std::vector<Mat4> pose_matrices = current_pose();
  // loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
  for (const auto &handle : handles)
  {
    if (!handle.enabled)
      continue;

    BoneIndex ik_bone_idx = handle.bone;
    Vec3 target_pos = handle.target;

    const auto &ik_bone = bones[ik_bone_idx];
    // pos_i(q) = T_i(q) * extent_i
    Vec3 tip_pos = (pose_matrices[ik_bone_idx] * Vec4(ik_bone.extent, 1.0f)).xyz();
    Vec3 error_vec = tip_pos - target_pos;

    BoneIndex current_bone_idx = ik_bone_idx;
    while (current_bone_idx != -1U)
    {
      const auto &bone = bones[current_bone_idx];
      Mat4 T = pose_matrices[current_bone_idx];
      Vec3 center = (T * Vec4(0.0f, 0.0f, 0.0f, 1.0f)).xyz();

      Vec3 x, y, z;
      bone.compute_rotation_axes(&x, &y, &z);

      Vec3 axis_x = (T * Vec4(1.0f, 0.0f, 0.0f, 0.0f)).xyz();
      Vec3 axis_y = (T * Mat4::angle_axis(-bone.pose.x, x) * Vec4(0.0f, 1.0f, 0.0f, 0.0f)).xyz();
      Vec3 axis_z = (T * Mat4::angle_axis(-bone.pose.x, x) * Mat4::angle_axis(-bone.pose.y, y) * Vec4(0.0f, 0.0f, 1.0f, 0.0f)).xyz();

      Vec3 partial_pos_x = cross(axis_x, tip_pos - center);
      Vec3 partial_pos_y = cross(axis_y, tip_pos - center);
      Vec3 partial_pos_z = cross(axis_z, tip_pos - center);

      gradient[current_bone_idx] += Vec3(dot(error_vec, partial_pos_x), dot(error_vec, partial_pos_y), dot(error_vec, partial_pos_z));
      //  std::cout << "partial_pos_x=" << partial_pos_x << ", "<< "partial_pos_y=" << partial_pos_y << ", " << "partial_pos_z=" << partial_pos_z << std::endl;
      //  std::cout << "gradient[current_bone_idx]=" << gradient[current_bone_idx] << std::endl;
      current_bone_idx = bone.parent;
    }
  }
  return {gradient, pose_matrices};
}

bool Skeleton::solve_ik(uint32_t steps)
{
  return solve_ik1(steps);
}

bool Skeleton::solve_ik2(uint32_t steps)
{
  // The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
  bool ret = true;

  // loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
  for (const auto &handle : handles)
  {
    if (!handle.enabled)
      continue;
    BoneIndex ik_bone_idx = handle.bone;
    float tau = 0.0f;
    for (uint32_t i = 0; i < steps; ++i)
    {
      auto [grads, matrices] = gradient_in_current_pose();
      float grad_norm_sq = 0.0f;

      BoneIndex current_bone_idx = ik_bone_idx;
      while (current_bone_idx != -1U)
      {
        grad_norm_sq += grads[current_bone_idx].norm_squared();
        current_bone_idx = bones[current_bone_idx].parent;
      }
      // if at a local minimum (e.g., gradient is near-zero), return 'true'.
      if (grad_norm_sq < 1e-8f)
      {
        break;
      }
      current_bone_idx = ik_bone_idx;
      while (current_bone_idx != -1U)
      {
        const auto &bone = bones[current_bone_idx];
        const auto &parent_bone = bones[bone.parent];
        if (bone.parent == -1U)
        {
          // Root bone: translate by base + base_offset, then apply rotation
          tau = 0.1f;
        }
        else
        {
          // Child bone: parent transform * translation by parent's extent * rotation
          Vec3 pos = (matrices[current_bone_idx] * Vec4(bone.extent, 1.0f)).xyz();
          Vec3 parent_pos = (matrices[bone.parent] * Vec4(parent_bone.extent, 1.0f)).xyz();
          Vec3 gradient = grads[current_bone_idx];
          Vec3 parent_gradient = grads[bone.parent];
          Vec3 diff_pos = pos - parent_pos;
          Vec3 diff_gradient = gradient - parent_gradient;
          tau = std::abs(dot(diff_pos, diff_gradient) / diff_gradient.norm_squared());
          std::cout << "dot(diff_pos, diff_gradient)=" << dot(diff_pos, diff_gradient) << ", diff_gradient.norm_squared=" << diff_gradient.norm_squared() << ", tau=" << tau << std::endl;
        }

        bones[current_bone_idx].pose -= tau * grads[current_bone_idx];
        current_bone_idx = bone.parent;
      }
      ret = false;
    }
  }
  return ret;
}

bool Skeleton::solve_ik1(uint32_t steps)
{
  // A4T2b - gradient descent
  // check which handles are enabled
  bool any_enabled = false;
  for (const auto &handle : handles)
  {
    if (handle.enabled)
    {
      any_enabled = true;
      break;
    }
  }
  if (!any_enabled)
    return true;

  // A small fixed step size for gradient descent.
  float tau = 1.0f;
  // run `steps` iterations
  for (uint32_t i = 0; i < steps; ++i)
  {
    // call gradient_in_current_pose() to compute d loss / d pose
    auto [grad, matrices] = gradient_in_current_pose();
    float grad_norm_sq = 0.0f;
    for (const auto &g : grad)
    {
      grad_norm_sq += g.norm_squared();
    }
    // Check for convergence
    // if at a local minimum (e.g., gradient is near-zero), return 'true'.
    if (grad_norm_sq < 1e-8f)
    {
      return true;
    }
    // Apply gradient descent step
    for (size_t j = 0; j < bones.size(); ++j)
    {

      bones[j].pose -= tau * grad[j];
    }
  }
  // if run through all steps, return `false`.
  return false;
}

/**
 * return the closest point on line segment a-b to point p:
 */
Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p)
{
  // A4T3: bone weight computation (closest point helper)
  // Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)

  Vec3 ab = b - a;
  float len_sq = ab.norm_squared();

  // If the segment has zero length, it's a point.
  if (len_sq < 1e-9f)
    return a;

  // Project p onto the line defined by a and b by finding the parameter t.
  // t = dot(p - a, b - a) / ||b - a||^2
  float t = dot(p - a, ab) / len_sq;

  // Clamp t to the range [0, 1] to stay on the line segment.
  t = std::max(0.0f, std::min(1.0f, t));

  // The closest point is a + t * (b - a).
  return a + t * ab;
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const
{
  assert(mesh_);
  auto &mesh = *mesh_;

  // A4T3: bone weight computation
  // Visit every vertex and **set new values** in Vertex::bone_weights

  // Get bind pose transformations
  std::vector<Mat4> bind_transforms = bind_pose();

  for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi)
  {
    // Clear existing bone weights
    vi->bone_weights.clear();

    std::vector<float> raw_weights(bones.size(), 0.0f);

    // Calculate raw weights for each bone
    for (size_t bone_idx = 0; bone_idx < bones.size(); ++bone_idx)
    {
      const Bone &bone = bones[bone_idx];

      // Get bone start and end positions in bind pose
      Vec3 bone_start = (bind_transforms[bone_idx] * Vec4(0.0f, 0.0f, 0.0f, 1.0f)).xyz();
      Vec3 bone_end = (bind_transforms[bone_idx] * Vec4(bone.extent, 1.0f)).xyz();

      // Find closest point on bone to vertex
      Vec3 closest_point = closest_point_on_line_segment(bone_start, bone_end, vi->position);

      // Calculate distance from vertex to closest point on bone
      float distance = (vi->position - closest_point).norm();

      // Calculate raw weight based on bone radius
      if (distance < bone.radius)
      {
        raw_weights[bone_idx] = (bone.radius - distance) / bone.radius;
      }
    }

    // Normalize weights so they sum to 1
    float total_weight = 0.0f;
    for (float weight : raw_weights)
    {
      total_weight += weight;
    }

    // Only store non-zero weights
    if (total_weight > 0.0f)
    {
      for (uint32_t bone_idx = 0; bone_idx < bones.size(); ++bone_idx)
      {
        if (raw_weights[bone_idx] > 0.0f)
        {
          float normalized_weight = raw_weights[bone_idx] / total_weight;
          vi->bone_weights.emplace_back(Halfedge_Mesh::Vertex::Bone_Weight{bone_idx, normalized_weight});
        }
      }
    }
    // If all weights are zero, leave bone_weights empty (vertex will use identity transform)
  }
}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector<Mat4> const &bind, std::vector<Mat4> const &current)
{
  assert(bind.size() == current.size());

  // A4T3: linear blend skinning

  // one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
  //  then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

  //---- step 1: figure out skinned positions ---

  std::unordered_map<Halfedge_Mesh::VertexCRef, Vec3> skinned_positions;
  std::unordered_map<Halfedge_Mesh::HalfedgeCRef, Vec3> skinned_normals;
  // reserve hash table space to (one hopes) avoid re-hashing:
  skinned_positions.reserve(mesh.vertices.size());
  skinned_normals.reserve(mesh.halfedges.size());

  //(you will probably want to precompute some bind-to-current transformation matrices here)
  std::vector<Mat4> bone_to_posed_transforms;
  bone_to_posed_transforms.reserve(bind.size());
  for (size_t i = 0; i < bind.size(); ++i)
  {
    bone_to_posed_transforms.push_back(current[i] * bind[i].inverse());
  }

  for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi)
  {
    // PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
    // skinned_positions.emplace(vi, vi->position);
    // NOTE: vertices with empty bone_weights should remain in place.
    Mat4 skin_transform; // Default constructor is identity
    if (!vi->bone_weights.empty())
    {
      skin_transform = Mat4::Zero;
      for (const auto &bw : vi->bone_weights)
      {
        if (bw.bone < bone_to_posed_transforms.size())
        {
          skin_transform += bw.weight * bone_to_posed_transforms[bw.bone];
        }
      }
    }
    skinned_positions.emplace(vi, (skin_transform * Vec4(vi->position, 1.0f)).xyz());
    // circulate corners at this vertex:
    Mat4 normal_transform = skin_transform.inverse().T();
    auto h = vi->halfedge;
    do
    {
      // could skip if h->face->boundary, since such corners don't get emitted
      if (!h->face->boundary)
      {
        skinned_normals.emplace(h, (normal_transform * Vec4(h->corner_normal, 0.0f)).xyz().unit());
      }
      h = h->twin->next;
    } while (h != vi->halfedge);
  }

  //---- step 2: transform into an indexed mesh ---

  // Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.
  // PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.
  // Indexed_Mesh result = Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);

  std::vector<Indexed_Mesh::Vert> verts;
  std::vector<Indexed_Mesh::Index> idxs;

  for (auto f_it = mesh.faces.begin(); f_it != mesh.faces.end(); ++f_it)
  {
    if (f_it->boundary)
      continue;

    uint32_t corners_begin = static_cast<uint32_t>(verts.size());
    auto h = f_it->halfedge;
    do
    {
      Indexed_Mesh::Vert vert;
      vert.pos = skinned_positions.at(h->vertex);
      vert.norm = skinned_normals.at(h);
      vert.uv = h->corner_uv;
      vert.id = f_it->id;
      verts.emplace_back(vert);
      h = h->next;
    } while (h != f_it->halfedge);

    uint32_t corners_end = static_cast<uint32_t>(verts.size());

    // Triangulate face into a fan
    for (uint32_t i = corners_begin + 1; i + 1 < corners_end; ++i)
    {
      idxs.emplace_back(corners_begin);
      idxs.emplace_back(i);
      idxs.emplace_back(i + 1);
    }
  }

  return Indexed_Mesh(std::move(verts), std::move(idxs));
}

void Skeleton::for_bones(const std::function<void(Bone &)> &f)
{
  for (auto &bone : bones)
  {
    f(bone);
  }
}

void Skeleton::erase_bone(BoneIndex bone)
{
  assert(bone < bones.size());
  // update indices in bones:
  for (uint32_t b = 0; b < bones.size(); ++b)
  {
    if (bones[b].parent == -1U)
      continue;
    if (bones[b].parent == bone)
    {
      assert(b > bone); // topological sort!
      // keep bone tips in the same place when deleting parent bone:
      bones[b].extent += bones[bone].extent;
      bones[b].parent = bones[bone].parent;
    }
    else if (bones[b].parent > bone)
    {
      assert(b > bones[b].parent); // topological sort!
      bones[b].parent -= 1;
    }
  }
  // erase the bone
  bones.erase(bones.begin() + bone);
  // update indices in handles (and erase any handles on this bone):
  for (uint32_t h = 0; h < handles.size(); /* later */)
  {
    if (handles[h].bone == bone)
    {
      erase_handle(h);
    }
    else if (handles[h].bone > bone)
    {
      handles[h].bone -= 1;
      ++h;
    }
    else
    {
      ++h;
    }
  }
}

void Skeleton::erase_handle(HandleIndex handle)
{
  assert(handle < handles.size());

  // nothing internally refers to handles by index so can just delete:
  handles.erase(handles.begin() + handle);
}

Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent)
{
  assert(parent == -1U || parent < bones.size());
  Bone bone;
  bone.extent = extent;
  bone.parent = parent;
  // all other parameters left as default.

  // slightly unfortunate hack:
  //(to ensure increasing IDs within an editing session, but reset on load)
  std::unordered_set<uint32_t> used;
  for (auto const &b : bones)
  {
    used.emplace(b.channel_id);
  }
  while (used.count(next_bone_channel_id))
    ++next_bone_channel_id;
  bone.channel_id = next_bone_channel_id++;

  // all other parameters left as default.

  BoneIndex index = BoneIndex(bones.size());
  bones.emplace_back(bone);

  return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target)
{
  assert(bone < bones.size());
  Handle handle;
  handle.bone = bone;
  handle.target = target;
  // all other parameters left as default.

  // slightly unfortunate hack:
  //(to ensure increasing IDs within an editing session, but reset on load)
  std::unordered_set<uint32_t> used;
  for (auto const &h : handles)
  {
    used.emplace(h.channel_id);
  }
  while (used.count(next_handle_channel_id))
    ++next_handle_channel_id;
  handle.channel_id = next_handle_channel_id++;

  HandleIndex index = HandleIndex(handles.size());
  handles.emplace_back(handle);

  return index;
}

Skeleton Skeleton::copy()
{
  // turns out that there aren't any fancy pointer data structures to fix up here.
  return *this;
}

void Skeleton::make_valid()
{
  for (uint32_t b = 0; b < bones.size(); ++b)
  {
    if (!(bones[b].parent == -1U || bones[b].parent < b))
    {
      warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
      bones[b].parent = -1U;
    }
  }
  if (bones.empty() && !handles.empty())
  {
    warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
    handles.clear();
  }
  for (uint32_t h = 0; h < handles.size(); ++h)
  {
    if (handles[h].bone >= HandleIndex(bones.size()))
    {
      warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
      handles[h].bone = 0;
    }
  }
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const
{
  return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const
{
  return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy()
{
  return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
