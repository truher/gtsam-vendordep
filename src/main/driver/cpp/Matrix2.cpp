#include <gtsam/base/Matrix.h>

extern "C" {
gtsam::Matrix2* Matrix2(     //
    double R11, double R12,  //
    double R21, double R22) {
    gtsam::Matrix2* m = new gtsam::Matrix2();
    (*m) << R11, R12,  //
        R21, R22;
    return m;
}
void Matrix2_delete(gtsam::Matrix2* p) {
    delete p;
}
double Matrix2_at(const gtsam::Matrix2* m, int r, int c) {
    return (*m)(r, c);
}
gtsam::Matrix2* Matrix2_unaryMinus(gtsam::Matrix2* m) {
    return new gtsam::Matrix2(-(*m));
}
gtsam::Matrix2* Matrix2_identity() {
    return new gtsam::Matrix2(gtsam::Matrix2::Identity());
}
gtsam::Matrix2* Matrix2_compose(const gtsam::Matrix2* a,
                                const gtsam::Matrix2* b) {
    return new gtsam::Matrix2((*a) * (*b));
}
gtsam::Matrix2* Matrix2_plus(const gtsam::Matrix2* v,
                             const gtsam::Matrix2* other) {
    return new gtsam::Matrix2((*v) + (*other));
}
gtsam::Matrix2* Matrix2_times(const gtsam::Matrix2* v, double a) {
    return new gtsam::Matrix2((*v) * a);
}
}