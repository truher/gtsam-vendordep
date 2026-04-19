#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>

extern "C" {
gtsam::Key symbol_shorthand_X(std::uint64_t j) {
  return gtsam::symbol_shorthand::X(j);
}
}