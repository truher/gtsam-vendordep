#include <gtsam/inference/Factor.h>

extern "C" {
const gtsam::KeyVector* Factor_keys(const gtsam::Factor* p) {
    return new gtsam::KeyVector(p->keys());
}

}