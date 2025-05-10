#include "test.h"
#include "lib/log.h"
#include "geometry/util.h"
#include "util/rand.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <vector>



Test test_me_b0("me.b0", []() {
  RNG rng;
  // 测试 acos(sqrt(u1)) 的分布
  std::vector<float> hist_correct(10, 0);
  for (int i = 0; i < 100000; ++i) {
      float u1 = rng.unit();
      float theta = std::acos(std::sqrt(u1));
      int bin = static_cast<int>(theta / (M_PI/2 * 0.1));
      hist_correct[bin]++;
  }


  // 测试 acos(u1) 的分布
  std::vector<float> hist_wrong(10, 0);
  for (int i = 0; i < 100000; ++i) {
      float u1 = rng.unit();
      float theta = std::acos(u1);
      int bin = static_cast<int>(theta / (M_PI/2 * 0.1));
      hist_wrong[bin]++;
  }

  std::cout << "\n--------" << std::endl;
  std::copy(hist_correct.begin(), hist_correct.end(),
              std::ostream_iterator<float>(std::cout, " "));
  std::cout << "\n--------" << std::endl;
  std::copy(hist_wrong.begin(), hist_wrong.end(),
              std::ostream_iterator<float>(std::cout, " "));
});