#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/linear/GaussianFactor.h>

extern "C" {
struct PtrPair {
    void* first;
    void* second;
};
PtrPair GaussianFactor_jacobian(gtsam::GaussianFactor::shared_ptr* f) {
    std::pair<gtsam::Matrix, gtsam::Vector> p = (*f)->jacobian();
    return {new gtsam::Matrix(p.first), new gtsam::Vector(p.second)};
}
}