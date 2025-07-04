#include "test.h"
#include "scene/skeleton.h"

inline std::vector<Skeleton::BoneIndex> setup_bones_current(Skeleton& skeleton) {

	skeleton.bones.clear();

	auto root = 	skeleton.add_bone(-1U, 		Vec3(1.0f, 0.0f, 0.0f));
	auto child1 = 	skeleton.add_bone(root, 	Vec3(0.0f, 1.0f, 0.0f));

  skeleton.bones[root].pose = Vec3(90.0f, 0.0f, 0.0f);
  skeleton.bones[child1].pose = Vec3(0.0f, 90.0f, 0.0f);

	skeleton.base = Vec3(0.0f, 0.0f, 1.0f);

	return {root, child1};
}

Test test_a4_task2_pose_current_simple("a4.task2.pose.current.simple", []() {
	Skeleton skeleton;
	auto joints = setup_bones_current(skeleton);
  std::vector<Mat4> expected;

	Mat4 expected_root = Mat4{Vec4{-0.000000f, 1.000000f, 0.000000f, 0.000000f}, 
                              Vec4{-1.000000f, -0.000000f, 0.000000f, 0.000000f}, 
                              Vec4{0.000000f, 0.000000f, 1.000000f, 0.000000f}, 
                              Vec4{0.000000f, 0.000000f, 1.000000f, 1.000000f}};
	Mat4 expected_child1 = Mat4{Vec4{0.000000f, -0.000000f, -1.000000f, 0.000000f}, 
                                Vec4{-1.000000f, -0.000000f, 0.000000f, 0.000000f}, 
                                Vec4{-0.000000f, 1.000000f, -0.000000f, 0.000000f}, 
                                Vec4{-0.000000f, 1.000000f, 1.000000f, 1.000000f}};
	
  expected.push_back(expected_root);
  expected.push_back(expected_child1);

  std::vector<Mat4> actual = skeleton.current_pose();

	if (Test::differs(expected[0], actual[0])) {
    std::cout << "Expected root joint matrix:\n" << expected[0] << "\n";
    std::cout << "Actual root joint matrix:\n" << actual[0] << "\n";
		throw Test::error("Test failed on the root!");
	}
	if (Test::differs(expected[1], actual[1])) {
    std::cout << "Expected root joint matrix:\n" << expected[1] << "\n";
    std::cout << "Actual root joint matrix:\n" << actual[1] << "\n";
		throw Test::error("Test failed on the first child joint!");
	}
});
