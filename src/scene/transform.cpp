
#include "transform.h"

Mat4 Transform::local_to_parent() const {
	return Mat4::translate(translation) * rotation.to_mat() * Mat4::scale(scale);
}

Mat4 Transform::parent_to_local() const {
	return Mat4::scale(1.0f / scale) * rotation.inverse().to_mat() * Mat4::translate(-translation);
}

Mat4 Transform::local_to_world() const {
	 // Start with the local-to-parent transformation
    Mat4 local_to_world_mat = local_to_parent();

    // If there is a parent, multiply by the parent's local-to-world matrix
    if (auto parent_ptr = parent.lock()) {
        local_to_world_mat = parent_ptr->local_to_world() * local_to_world_mat;
    }

    return local_to_world_mat;
}

Mat4 Transform::world_to_local() const {
	// Start with the parent-to-local transformation
    Mat4 world_to_local_mat = parent_to_local();

    // If there is a parent, multiply by the parent's world-to-local matrix
    if (auto parent_ptr = parent.lock()) {
        world_to_local_mat = world_to_local_mat * parent_ptr->world_to_local();
    }

    return world_to_local_mat;
}

bool operator!=(const Transform& a, const Transform& b) {
	return a.parent.lock() != b.parent.lock() || a.translation != b.translation ||
	       a.rotation != b.rotation || a.scale != b.scale;
}
