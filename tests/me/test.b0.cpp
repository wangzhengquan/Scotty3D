#include "test.h"
#include "lib/log.h"
#include "geometry/util.h"
#include <iostream>


Test test_me_b0("me.b0", []() {
  assert2(true, "a=%d, b=%d", 1, 2);
  Indexed_Mesh index_mesh = Util::pentagon_mesh(1.0f);
});