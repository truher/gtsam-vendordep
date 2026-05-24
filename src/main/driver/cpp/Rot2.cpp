#include <gtsam/geometry/Rot2.h>

extern "C" {
gtsam::Rot2* Rot2(double theta) {
    return new gtsam::Rot2(theta);
}
void Rot2_delete(gtsam::Rot2* p) {
    delete p;
}
double Rot2_theta(const gtsam::Rot2* p) {
    return p->theta();
}
double Rot2_c(const gtsam::Rot2* p) {
    return p->c();
}
double Rot2_s(const gtsam::Rot2* p) {
    return p->s();
}
gtsam::Matrix2* Rot2_matrix(const gtsam::Rot2* p) {
    return new gtsam::Matrix2(p->matrix());
}
gtsam::Rot2* Rot2_compose(const gtsam::Rot2* a, const gtsam::Rot2* b) {
    return new gtsam::Rot2((*a) * (*b));
}
gtsam::Point2* Rot2_rotate(const gtsam::Rot2* r, const gtsam::Point2* p) {
    return new gtsam::Point2(r->rotate((*p)));
}
gtsam::Rot2* Rot2_fromCosSin(double c, double s) {
    return new gtsam::Rot2(gtsam::Rot2::fromCosSin(c, s));
}
gtsam::Rot2* Rot2_atan2(double y, double x) {
    return new gtsam::Rot2(gtsam::Rot2::atan2(y, x));
}
gtsam::Point2* Rot2_unit(const gtsam::Rot2* r) {
    return new gtsam::Point2(r->unit());
}
}