#include <gtsam/geometry/Rot3.h>

extern "C" {
gtsam::Rot3* Rot3Point3(const gtsam::Point3* col1,  //
                        const gtsam::Point3* col2,  //
                        const gtsam::Point3* col3) {
    return new gtsam::Rot3(*col1, *col2, *col3);
}
/// Construct from a rotation matrix, as doubles in *row-major* order !!!
gtsam::Rot3* Rot3(                       //
    double R11, double R12, double R13,  //
    double R21, double R22, double R23,  //
    double R31, double R32, double R33) {
    return new gtsam::Rot3(R11, R12, R13,  //
                           R21, R22, R23,  //
                           R31, R32, R33);
}
gtsam::Rot3* Rot3Matrix3(gtsam::Matrix3* R) {
    return new gtsam::Rot3(*R);
}
void Rot3_delete(gtsam::Rot3* p) {
    delete p;
}
gtsam::Rot3* Rot3_Ypr(double y, double p, double r) {
    return new gtsam::Rot3(gtsam::Rot3::Ypr(y, p, r));
}
gtsam::Rot3* Rot3_Rodrigues(double wx, double wy, double wz) {
    return new gtsam::Rot3(gtsam::Rot3::Rodrigues(wx, wy, wz));
}
gtsam::Rot3* Rot3_AxisAngle(const gtsam::Point3* axis, double angle) {
    return new gtsam::Rot3(gtsam::Rot3::AxisAngle(*axis, angle));
}
gtsam::Rot3* Rot3_compose(const gtsam::Rot3* a, const gtsam::Rot3* b) {
    return new gtsam::Rot3((*a) * (*b));
}
gtsam::Rot3* Rot3_between(const gtsam::Rot3* r, const gtsam::Rot3* g) {
    return new gtsam::Rot3(r->between(*g));
}
gtsam::Rot3* Rot3_inverse(const gtsam::Rot3* r) {
    return new gtsam::Rot3(r->inverse());
}
gtsam::Vector3* Rot3_localCoordinates(const gtsam::Rot3* r,
                                      const gtsam::Rot3* g) {
    return new gtsam::Vector3(r->localCoordinates(*g));
}
gtsam::Rot3* Rot3_retract(const gtsam::Rot3* r, const gtsam::Vector3* v) {
    return new gtsam::Rot3(r->retract(*v));
}
gtsam::Vector3* Rot3_logmap(const gtsam::Rot3* r, const gtsam::Rot3* g) {
    return new gtsam::Vector3(r->logmap(*g));
}
gtsam::Rot3* Rot3_expmap(const gtsam::Rot3* r, const gtsam::Vector3* v) {
    return new gtsam::Rot3(r->expmap(*v));
}
gtsam::Vector3* Rot3_Logmap(const gtsam::Rot3* p) {
    return new gtsam::Vector3(gtsam::Rot3::Logmap(*p));
}
gtsam::Rot3* Rot3_Expmap(const gtsam::Vector3* xi) {
    return new gtsam::Rot3(gtsam::Rot3::Expmap(*xi));
}
bool Rot3_check_group_invariants(const gtsam::Rot3* a,  //
                                 const gtsam::Rot3* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Rot3_check_manifold_invariants(const gtsam::Rot3* a,
                                    const gtsam::Rot3* b) {
    return gtsam::check_manifold_invariants(*a, *b);
}
}