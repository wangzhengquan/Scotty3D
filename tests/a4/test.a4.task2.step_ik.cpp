#include "test.h"
#include "scene/skeleton.h"
#include <queue>

static bool in_range(Vec3 testee, Vec3 lower, Vec3 upper) {
	return testee.x >= lower.x && testee.x <= upper.x && testee.y >= lower.y && testee.y <= upper.y &&
	       testee.z >= lower.z && testee.z <= upper.z;
}

Test test_a4_task2_step_ik_single_joint_single_target("a4.task2.step_ik.single_joint.single_target", []() {
	Skeleton simple;
	auto joint = simple.add_bone(-1U, Vec3(0.0f, 1.0f, 0.0f));
	auto ikHandle = simple.add_handle(joint, Vec3(0.0f, 1.0f, 1.0f));
	simple.handles[ikHandle].enabled = true;

	std::queue<Vec3> queue;

	for (int32_t i = 0; i < 100; i++) {
		simple.solve_ik(100);
		if (queue.size() > 1) {
			queue.pop();
		}
		queue.push(simple.bones[joint].pose);
	}

	if (Test::differs(queue.front(), queue.back())) {
    std::cout << "\nqueue.front=" << queue.front() << ", queue.back=" << queue.back() << std::endl;
		throw Test::error("IK did not converge within the desired number of iterations!");
	}
	if (Test::differs(simple.base, Vec3(0.0f, 0.0f, 0.0f))) {
    std::cout << "\nsimple.base=" << simple.base << std::endl;
		throw Test::error("Base position should not move during IK!");
	}
	if (!in_range(simple.bones[joint].pose, Vec3(44.9f, 0.0f, 0.0f), Vec3(45.1f, 0.0f, 0.0f))) {
		std::cout << "\nsimple.bones[joint].pose=" << simple.bones[joint].pose << std::endl;
    throw Test::error("Joint pose differs from target!");
	}
});


Test test_a4_task2_step_ik_single_joint_single_target2("a4.task2.step_ik.single_joint.single_target2", []() {
	Skeleton simple;
	auto joint = simple.add_bone(-1U, Vec3(0.0f, 1.0f, 0.0f));
	auto ikHandle = simple.add_handle(joint, Vec3(0.0f, 1.0f, 1.0f));
	simple.handles[ikHandle].enabled = true;

	simple.solve_ik(360);

	if (Test::differs(simple.base, Vec3(0.0f, 0.0f, 0.0f))) {
    std::cout << "\nsimple.base=" << simple.base << std::endl;
		throw Test::error("Base position should not move during IK!");
	}
	if (!in_range(simple.bones[joint].pose, Vec3(44.9f, 0.0f, 0.0f), Vec3(45.1f, 0.0f, 0.0f))) {
		std::cout << "\nsimple.bones[joint].pose=" << simple.bones[joint].pose << std::endl;
    throw Test::error("Joint pose differs from target!");
	}
});

