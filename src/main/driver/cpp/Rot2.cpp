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
gtsam::Rot2* Rot2_composeH(const gtsam::Rot2* a, const gtsam::Rot2* b,
                           gtsam::Matrix* H1, gtsam::Matrix* H2) {
    return new gtsam::Rot2(a->compose(*b, *H1, *H2));
}
gtsam::Point2* Rot2_rotate(const gtsam::Rot2* r, const gtsam::Point2* p) {
    return new gtsam::Point2(r->rotate((*p)));
}
gtsam::Point2* Rot2_rotateH(const gtsam::Rot2* r, const gtsam::Point2* p,
                            gtsam::Matrix* H1, gtsam::Matrix* H2) {
    return new gtsam::Point2(r->rotate(*p, *H1, *H2));
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
gtsam::Rot2* Rot2_inverse(const gtsam::Rot2* r) {
    return new gtsam::Rot2(r->inverse());
}
gtsam::Matrix2* Rot2_transpose(const gtsam::Rot2* r) {
    return new gtsam::Matrix2(r->transpose());
}
// see Lie.h
gtsam::Rot2* Rot2_between(const gtsam::Rot2* r, const gtsam::Rot2* g) {
    return new gtsam::Rot2(r->between(*g));
}
// see Lie.h
gtsam::Rot2* Rot2_betweenH(const gtsam::Rot2* r, const gtsam::Rot2* g,
                           gtsam::Matrix* H1, gtsam::Matrix* H2) {
    return new gtsam::Rot2(r->between(*g, *H1, *H2));
}
gtsam::Rot2* Rot2_retract(const gtsam::Rot2* r, const gtsam::Vector1* v) {
    return new gtsam::Rot2(r->retract(*v));
}
gtsam::Vector1* Rot2_localCoordinates(const gtsam::Rot2* r,
                                      const gtsam::Rot2* g) {
    return new gtsam::Vector1(r->localCoordinates(*g));
}
gtsam::Vector1* Rot2_Logmap(const gtsam::Rot2* p) {
    return new gtsam::Vector1(gtsam::Rot2::Logmap(*p));
}
gtsam::Rot2* Rot2_Expmap(const gtsam::Vector1* xi) {
    return new gtsam::Rot2(gtsam::Rot2::Expmap(*xi));
}
}