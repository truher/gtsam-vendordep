#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/GaussianFactor.h>

#include "pairs.h"

extern "C" {
PtrPair GaussianFactor_jacobian(gtsam::GaussianFactor* f) {
    std::pair<gtsam::Matrix, gtsam::Vector> p = f->jacobian();
    return {new gtsam::Matrix(p.first), new gtsam::Vector(p.second)};
}
}