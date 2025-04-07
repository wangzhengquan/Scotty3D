#include "test.h"
#include "lib/log.h"
#include "geometry/util.h"
#include <iostream>


Test test_me_b0("me.b0", []() {
  Indexed_Mesh index_mesh = Util::pentagon_mesh(1.0f);
});