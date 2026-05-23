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
gtsam::Pose2* Pose2Vector3(const gtsam::Vector3* v) {
    return new gtsam::Pose2(gtsam::Vector(*v));
}
// TODO: make this Vector3 somehow
gtsam::Pose2* Pose2_retract(const gtsam::Pose2* p, const gtsam::Vector* v) {
    return new gtsam::Pose2(p->retract(*v));
}
/** This is from the LieGroup trait, see gtsam/base/Lie.h */
// TODO: figure out Vector vs Vector3 here.
gtsam::Pose2* Pose2_Retract(const gtsam::Pose2* origin,  //
                            const gtsam::Vector* v,      //
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
gtsam::Pose2* Pose2_Expmap(const gtsam::Vector* xi) {
    gtsam::Vector3 xi3(*xi);
    std::cout << "xi3 " << xi3 << std::endl;
    gtsam::Pose2 p = gtsam::Pose2::Expmap(xi3);
    std::cout << "p " << p << std::endl;
    return new gtsam::Pose2(p);
}
gtsam::Pose2* Pose2_ExpmapH(const gtsam::Vector* xi, gtsam::Matrix3* Hv) {
    std::cout << "xi " << *xi << std::endl;
    return new gtsam::Pose2(gtsam::Pose2::Expmap(gtsam::Vector3(*xi), *Hv));
}
gtsam::Vector* Pose2_Logmap(const gtsam::Pose2* p) {
    return new gtsam::Vector(gtsam::Pose2::Logmap(*p));
}
gtsam::Vector* Pose2_LogmapH(const gtsam::Pose2* p, gtsam::Matrix3* H) {
    return new gtsam::Vector(gtsam::Pose2::Logmap(*p, *H));
}
gtsam::Vector3* Pose2_logmap(const gtsam::Pose2* p0, const gtsam::Pose2* p1) {
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
gtsam::Vector* Pose2_logmap_default(const gtsam::Pose2* a,
                                    const gtsam::Pose2* b) {
    return new gtsam::Vector(gtsam::logmap_default(*a, *b));
}
gtsam::Pose2* Pose2_expmap_default(const gtsam::Pose2* p,
                                   const gtsam::Vector* d) {
    return new gtsam::Pose2(gtsam::expmap_default(*p, *d));
}
gtsam::Point2* Pose2_transformTo(const gtsam::Pose2* p,
                                 const gtsam::Point2* point,
                                 gtsam::Matrix* Dpose, gtsam::Matrix* Dpoint) {
    return new gtsam::Point2(p->transformTo(*point, *Dpose, *Dpoint));
}
gtsam::Point2* Pose2_transformFrom(const gtsam::Pose2* p,
                                   const gtsam::Point2* point,
                                   gtsam::Matrix* Dpose,
                                   gtsam::Matrix* Dpoint) {
    return new gtsam::Point2(p->transformFrom(*point, *Dpose, *Dpoint));
}
gtsam::Matrix3* Pose2_ExpmapDerivative(const gtsam::Vector3* v) {
    return new gtsam::Matrix3(gtsam::Pose2::ExpmapDerivative(*v));
}
gtsam::Matrix* Pose2_Hat(const gtsam::Vector* v) {
    // TODO: make this work for Matrix3 and Vector3
    return new gtsam::Matrix(gtsam::Pose2::Hat(*v));
}
gtsam::Vector* Pose2_Vee(const gtsam::Matrix* X) {
    // TODO: make this work for Matrix3 and Vector3
    return new gtsam::Vector(gtsam::Pose2::Vee(*X));
}
gtsam::Pose2* Pose2_expm(const gtsam::Vector* x) {
    std::cout << "x " << *x << std::endl;
    gtsam::Matrix xhat = gtsam::wedge<gtsam::Pose2>(*x);
    std::cout << "xhat " << xhat << std::endl;
    return new gtsam::Pose2(gtsam::expm<gtsam::Pose2>(*x));
}
}