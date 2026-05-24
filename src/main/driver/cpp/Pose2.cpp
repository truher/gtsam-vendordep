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
    return new gtsam::Pose2(gtsam::Matrix(*T));
}
gtsam::Pose2* Pose2Vector3(const gtsam::Vector3* v) {
    return new gtsam::Pose2(gtsam::Vector(*v));
}
// TODO: make this Vector3 somehow
gtsam::Pose2* Pose2_retract(const gtsam::Pose2* p, const gtsam::Vector3* v) {
    return new gtsam::Pose2(p->retract(*v));
}
/** This is from the LieGroup trait, see gtsam/base/Lie.h */
gtsam::Pose2* Pose2_Retract(const gtsam::Pose2* origin,  //
                            const gtsam::Vector3* v,     //
                            gtsam::Matrix* Horigin,      //
                            gtsam::Matrix* Hv) {
    return new gtsam::Pose2(origin->retract(gtsam::Vector3(*v), *Horigin, *Hv));
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
gtsam::Vector3* Pose2_localCoordinates(const gtsam::Pose2* p,
                                       const gtsam::Pose2* g) {
    return new gtsam::Vector3(p->localCoordinates(*g));
}
gtsam::Pose2* Pose2_between(const gtsam::Pose2* a, const gtsam::Pose2* b) {
    return new gtsam::Pose2(a->between(*b));
}
gtsam::Pose2* Pose2_betweenH(const gtsam::Pose2* a, const gtsam::Pose2* b,
                             gtsam::Matrix* H1, gtsam::Matrix* H2) {
    return new gtsam::Pose2(a->between(*b, *H1, *H2));
}
gtsam::Pose2* Pose2_inverse(const gtsam::Pose2* p) {
    return new gtsam::Pose2(p->inverse());
}
gtsam::Pose2* Pose2_inverseH(const gtsam::Pose2* p, gtsam::Matrix* H) {
    return new gtsam::Pose2(p->inverse(*H));
}
/** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
gtsam::Matrix* Pose2_AdjointMap(const gtsam::Pose2* p) {
    return new gtsam::Matrix(p->AdjointMap());
}
gtsam::Vector3* Pose2_Adjoint(const gtsam::Pose2* p, const gtsam::Vector3* v) {
    return new gtsam::Vector3(p->Adjoint(*v));
}
gtsam::Pose2* Pose2_Expmap(const gtsam::Vector3* xi) {
    gtsam::Vector3 xi3(*xi);
    gtsam::Pose2 p = gtsam::Pose2::Expmap(xi3);
    return new gtsam::Pose2(p);
}
gtsam::Pose2* Pose2_ExpmapH(const gtsam::Vector3* xi, gtsam::Matrix* Hv) {
    return new gtsam::Pose2(gtsam::Pose2::Expmap(gtsam::Vector3(*xi), *Hv));
}
gtsam::Vector3* Pose2_Logmap(const gtsam::Pose2* p) {
    return new gtsam::Vector3(gtsam::Pose2::Logmap(*p));
}
gtsam::Vector3* Pose2_LogmapH(const gtsam::Pose2* p, gtsam::Matrix* H) {
    return new gtsam::Vector3(gtsam::Pose2::Logmap(*p, *H));
}
gtsam::Vector3* Pose2_logmap(const gtsam::Pose2* p0, const gtsam::Pose2* p1) {
    return new gtsam::Vector3(p0->logmap(*p1));
}
gtsam::Pose2* Pose2_compose(const gtsam::Pose2* a, const gtsam::Pose2* b) {
    return new gtsam::Pose2((*a) * (*b));
}
gtsam::Pose2* Pose2_composeH(const gtsam::Pose2* a,  //
                             const gtsam::Pose2* b,  //
                             gtsam::Matrix* H1,      //
                             gtsam::Matrix* H2) {
    return new gtsam::Pose2(a->compose(*b, *H1, *H2));
}
gtsam::Matrix3* Pose2_matrix(const gtsam::Pose2* p) {
    return new gtsam::Matrix3(p->matrix());
}
gtsam::Vector3* Pose2_logmap_default(const gtsam::Pose2* a,
                                     const gtsam::Pose2* b) {
    return new gtsam::Vector3(gtsam::logmap_default(*a, *b));
}
gtsam::Pose2* Pose2_expmap_default(const gtsam::Pose2* p,
                                   const gtsam::Vector3* d) {
    return new gtsam::Pose2(gtsam::expmap_default(*p, *d));
}
gtsam::Point2* Pose2_transformTo(const gtsam::Pose2* p,  //
                                 const gtsam::Point2* point) {
    return new gtsam::Point2(p->transformTo(*point));
}
// the actual type here is OptionalJacobian which "ururps"
// (coercing) the supplied dynamic or fixed matrix .  so I could
// use the real OptionalJacobian type.  In the unit test,
// the universal pattern is to supply a dynamic array, and
// that's also what the python wrapper does: it occasionally
// uses fixed matrices, e.g. Matrix3 for expmap derivative,
// but otherwise uses dynamic matrices for everything. so I
// could do that.  Or I could make a bunch of fixed Matrix
// types, which seems like a lot of work for almost no reason?
gtsam::Point2* Pose2_transformToH(const gtsam::Pose2* p,       //
                                  const gtsam::Point2* point,  //
                                  gtsam::Matrix* Dpose,        //
                                  gtsam::Matrix* Dpoint) {
    return new gtsam::Point2(p->transformTo(*point, *Dpose, *Dpoint));
}
gtsam::Point2* Pose2_transformFrom(const gtsam::Pose2* p,
                                   const gtsam::Point2* point) {
    return new gtsam::Point2(p->transformFrom(*point));
}
gtsam::Point2* Pose2_transformFromH(const gtsam::Pose2* p,
                                    const gtsam::Point2* point,
                                    gtsam::Matrix* Dpose,
                                    gtsam::Matrix* Dpoint) {
    return new gtsam::Point2(p->transformFrom(*point, *Dpose, *Dpoint));
}
gtsam::Matrix3* Pose2_ExpmapDerivative(const gtsam::Vector3* v) {
    return new gtsam::Matrix3(gtsam::Pose2::ExpmapDerivative(*v));
}
gtsam::Point2* Pose2_translation(const gtsam::Pose2* p, gtsam::Matrix* H) {
    return new gtsam::Point2(p->translation(*H));
}
gtsam::Rot2* Pose2_bearingPoint2(const gtsam::Pose2* p, gtsam::Point2* pt) {
    return new gtsam::Rot2(p->bearing(*pt));
}
gtsam::Rot2* Pose2_bearingPose2(const gtsam::Pose2* p, gtsam::Pose2* p2) {
    return new gtsam::Rot2(p->bearing(*p2));
}
double Pose2_rangePoint2(const gtsam::Pose2* p, gtsam::Point2* pt) {
    return p->range(*pt);
}
double Pose2_rangePose2(const gtsam::Pose2* p, gtsam::Pose2* p2) {
    return p->range(*p2);
}
}