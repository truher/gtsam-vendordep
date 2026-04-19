#include <gtsam/base/Lie.h>
#include <gtsam/geometry/Pose2.h>

extern "C" {
gtsam::Pose2* Pose2(double x, double y, double theta) {
    return new gtsam::Pose2(x, y, theta);
}
void Pose2_delete(gtsam::Pose2* p) {
    delete p;
}
gtsam::Pose2* Pose2Rot2Point2(gtsam::Rot2* r, gtsam::Point2* t) {
    return new gtsam::Pose2(*r, *t);
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
gtsam::Pose2::TangentVector* Pose2_localCoordinates(const gtsam::Pose2* p,
                                                    const gtsam::Pose2* g) {
    return new gtsam::Pose2::TangentVector(p->localCoordinates(*g));
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
}