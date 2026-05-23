#include <gtsam/base/Matrix.h>

extern "C" {
gtsam::Matrix6* Matrix6() {
    gtsam::Matrix6* m = new gtsam::Matrix6();
    m->setZero();
    return m;
}
void Matrix6_delete(gtsam::Matrix6* p) {
    delete p;
}
double Matrix6_at(const gtsam::Matrix6* m, int r, int c) {
    return (*m)(r, c);
}
gtsam::Matrix6* Matrix6_unaryMinus(gtsam::Matrix6* m) {
    return new gtsam::Matrix6(-(*m));
}
gtsam::Matrix6* Matrix6_identity() {
    return new gtsam::Matrix6(gtsam::Matrix6::Identity());
}
bool Matrix6_equals(const gtsam::Matrix6* a, const gtsam::Matrix6* b, double tol) {
    return gtsam::equal_with_abs_tol(*a, *b, tol);
}
gtsam::Matrix6* Matrix6_compose(const gtsam::Matrix6* a, const gtsam::Matrix6* b) {
    return new gtsam::Matrix6((*a) * (*b));
}
gtsam::Matrix6* Matrix6_plus(const gtsam::Matrix6* v, const gtsam::Matrix6* other) {
    return new gtsam::Matrix6((*v) + (*other));
}
gtsam::Matrix6* Matrix6_times(const gtsam::Matrix6* v, double a) {
    return new gtsam::Matrix6((*v) * a);
}
gtsam::Matrix6* Matrix6_inverse(const gtsam::Matrix6* m) {
    std::cout << "Matrix6 inverse " << *m << std::endl;
    return new gtsam::Matrix6(m->inverse());
}
}