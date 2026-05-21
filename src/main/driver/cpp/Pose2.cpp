#include <gtsam/base/Lie.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

extern "C" {
gtsam::Pose2* Pose2(double x, double y, double theta) {
    return new gtsam::Pose2(x, y, theta);
}
void Pose2_delete(gtsam::Pose2* p) {
    delete p;
}
gtsam::Pose2* Pose2DoublePoint2(double theta, const gtsam::Point2* t) {
    return new gtsam::Pose2(theta, *t);
}
gtsam::Pose2* Pose2Rot2Point2(const gtsam::Rot2* r, const gtsam::Point2* t) {
    return new gtsam::Pose2(*r, *t);
}
gtsam::Pose2* Pose2Matrix3(const gtsam::Matrix3* T) {
    return new gtsam::Pose2((gtsam::Matrix)(*T));
}
// TODO: make this Vector3 somehow
gtsam::Pose2* Pose2_retract(const gtsam::Pose2* p, const gtsam::Vector* v) {
    return new gtsam::Pose2(p->retract(*v));
}
/** This is from the LieGroup trait, see gtsam/base/Lie.h */
gtsam::Pose2* Pose2_Retract(const gtsam::Pose2* origin,  //
                            const gtsam::Vector3* v,      //
                            gtsam::Matrix* Horigin,      //
                            gtsam::Matrix* Hv) {
    return new gtsam::Pose2(gtsam::Pose2::Retract(*origin, *v, *Horigin, *Hv));
}
double Pose2_x(const gtsam::Pose2* p) {
    return p->x();
}
double Pose2_y(const gtsam::Pose2* p) {
    return p->y();
}
double Pose2_theta(const gtsam::Pose2* p) {
    return p->theta();
}
gtsam::Point2* Pose2_t(const gtsam::Pose2* p) {
    return new gtsam::Point2(p->t());
}
gtsam::Rot2* Pose2_r(const gtsam::Pose2* p) {
    return new gtsam::Rot2(p->r());
}
// maybe this should be Vector3
gtsam::Vector* Pose2_localCoordinates(const gtsam::Pose2* p,
                                      const gtsam::Pose2* g) {
    return new gtsam::Vector(p->localCoordinates(*g));
}
gtsam::Pose2* Pose2_between(const gtsam::Pose2* a, const gtsam::Pose2* b) {
    return new gtsam::Pose2(a->between(*b));
}
gtsam::Pose2* Pose2_inverse(const gtsam::Pose2* p) {
    return new gtsam::Pose2(p->inverse());
}
gtsam::Matrix3* Pose2_AdjointMap(const gtsam::Pose2* p) {
    return new gtsam::Matrix3(p->AdjointMap());
}
// picks primitives out of xi, creates new Pose2
gtsam::Pose2* Pose2_Expmap(const gtsam::Vector3* xi) {
    return new gtsam::Pose2(gtsam::Pose2::Expmap(*xi));
}
gtsam::Vector3* Pose2_log(const gtsam::Pose2* p0, const gtsam::Pose2* p1) {
    return new gtsam::Vector3(p0->logmap(*p1));
}
void Pose2_print(const gtsam::Pose2* p) {
    p->print();
}
bool Pose2_equals(const gtsam::Pose2* a, const gtsam::Pose2* b, double tol) {
    return a->equals(*b, tol);
}
gtsam::Pose2* Pose2_compose(const gtsam::Pose2* a, const gtsam::Pose2* b) {
    return new gtsam::Pose2((*a) * (*b));
}
gtsam::Matrix3* Pose2_matrix(const gtsam::Pose2* p) {
    return new gtsam::Matrix3(p->matrix());
}
}