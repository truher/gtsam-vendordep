#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose2.h>

extern "C" {
gtsam::Pose2::TangentVector* TangentVector_unaryMinus(
    gtsam::Pose2::TangentVector* v) {
    return new gtsam::Pose2::TangentVector(-*v);
}
void TangentVector_delete(gtsam::Pose2::TangentVector* p) {
    delete p;
}
}