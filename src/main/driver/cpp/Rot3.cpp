#include <gtsam/geometry/Rot3.h>
#include "pairs.h"

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
gtsam::Rot3* Rot3Matrix3(const gtsam::Matrix3* R) {
    return new gtsam::Rot3(*R);
}
gtsam::Rot3* Rot3Quaternion(const gtsam::Quaternion* q) {
    return new gtsam::Rot3(*q);
}
gtsam::Rot3* Rot3_Quaternion(double w, double x, double y, double z) {
    return new gtsam::Rot3(gtsam::Rot3::Quaternion(w, x, y, z));
}
void Rot3_delete(gtsam::Rot3* p) {
    delete p;
}
gtsam::Rot3* Rot3_Yaw(double t) {
    return new gtsam::Rot3(gtsam::Rot3::Yaw(t));
}
gtsam::Rot3* Rot3_Pitch(double t) {
    return new gtsam::Rot3(gtsam::Rot3::Pitch(t));
}
gtsam::Rot3* Rot3_Roll(double t) {
    return new gtsam::Rot3(gtsam::Rot3::Roll(t));
}
gtsam::Rot3* Rot3_Ypr(double y, double p, double r) {
    return new gtsam::Rot3(gtsam::Rot3::Ypr(y, p, r));
}
gtsam::Rot3* Rot3_YprH(double y, double p, double r,  //
                       gtsam::Matrix* H1,             //
                       gtsam::Matrix* H2,             //
                       gtsam::Matrix* H3) {           //
    return new gtsam::Rot3(gtsam::Rot3::Ypr(y, p, r, *H1, *H2, *H3));
}
gtsam::Rot3* Rot3_Rodrigues(double wx, double wy, double wz) {
    return new gtsam::Rot3(gtsam::Rot3::Rodrigues(wx, wy, wz));
}
gtsam::Rot3* Rot3_RodriguesVector3(const gtsam::Point3* v) {
    return new gtsam::Rot3(gtsam::Rot3::Rodrigues(*v));
}
gtsam::Rot3* Rot3_AxisAnglePoint3(const gtsam::Point3* axis, double angle) {
    return new gtsam::Rot3(gtsam::Rot3::AxisAngle(*axis, angle));
}
gtsam::Rot3* Rot3_AxisAngleUnit3(const gtsam::Unit3* axis, double angle) {
    return new gtsam::Rot3(gtsam::Rot3::AxisAngle(*axis, angle));
}
gtsam::Matrix3* Rot3_matrix(const gtsam::Rot3* p) {
    return new gtsam::Matrix3(p->matrix());
}
gtsam::Rot3* Rot3_compose(const gtsam::Rot3* a,  //
                          const gtsam::Rot3* b) {
    return new gtsam::Rot3((*a) * (*b));
}
gtsam::Rot3* Rot3_composeH(const gtsam::Rot3* a,  //
                           const gtsam::Rot3* b,  //
                           gtsam::Matrix* H1,     //
                           gtsam::Matrix* H2) {   //
    return new gtsam::Rot3(a->compose(*b, *H1, *H2));
}
gtsam::Rot3* Rot3_between(const gtsam::Rot3* r, const gtsam::Rot3* g) {
    return new gtsam::Rot3(r->between(*g));
}
gtsam::Rot3* Rot3_betweenH(const gtsam::Rot3* r,  //
                           const gtsam::Rot3* g,  //
                           gtsam::Matrix* H1,     //
                           gtsam::Matrix* H2) {   //
    return new gtsam::Rot3(r->between(*g, *H1, *H2));
}
gtsam::Rot3* Rot3_inverse(const gtsam::Rot3* r) {
    return new gtsam::Rot3(r->inverse());
}
gtsam::Rot3* Rot3_inverseH(const gtsam::Rot3* p, gtsam::Matrix* H) {
    return new gtsam::Rot3(p->inverse(*H));
}
gtsam::Matrix* Rot3_AdjointMap(const gtsam::Rot3* r) {
    return new gtsam::Matrix(r->AdjointMap());
}
gtsam::Matrix3* Rot3_transpose(const gtsam::Rot3* r) {
    return new gtsam::Matrix3(r->transpose());
}
gtsam::Vector3* Rot3_localCoordinates(const gtsam::Rot3* r,
                                      const gtsam::Rot3* g) {
    return new gtsam::Vector3(r->localCoordinates(*g));
}
gtsam::Vector3* Rot3_localCoordinatesH(const gtsam::Rot3* r,  //
                                       const gtsam::Rot3* g,  //
                                       gtsam::Matrix* H1,     //
                                       gtsam::Matrix* H2) {   //
    return new gtsam::Vector3(r->localCoordinates(*g, *H1, *H2));
}
gtsam::Rot3* Rot3_retract(const gtsam::Rot3* r, const gtsam::Vector3* v) {
    return new gtsam::Rot3(r->retract(*v));
}
gtsam::Rot3* Rot3_retractH(const gtsam::Rot3* r,     //
                           const gtsam::Vector3* v,  //
                           gtsam::Matrix* H1,        //
                           gtsam::Matrix* H2) {      //
    return new gtsam::Rot3(r->retract(*v, *H1, *H2));
}
// gtsam::Vector3* Rot3_logmap(const gtsam::Rot3* r, const gtsam::Rot3* g) {
//     return new gtsam::Vector3(r->logmap(*g));
// }
// gtsam::Rot3* Rot3_expmap(const gtsam::Rot3* r, const gtsam::Vector3* v) {
//     return new gtsam::Rot3(r->expmap(*v));
// }
gtsam::Rot3* Rot3_expmapH(const gtsam::Rot3* r,     //
                          const gtsam::Vector3* v,  //
                          gtsam::Matrix* H1,        //
                          gtsam::Matrix* H2) {      //
    return new gtsam::Rot3(r->expmap(*v, *H1, *H2));
}
gtsam::Vector3* Rot3_logmapH(const gtsam::Rot3* r,  //
                             const gtsam::Rot3* g,  //
                             gtsam::Matrix* H1,     //
                             gtsam::Matrix* H2) {   //
    return new gtsam::Vector3(r->logmap(*g, *H1, *H2));
}

gtsam::Rot3* Rot3_OriginRetract(const gtsam::Vector3* v) {
    return new gtsam::Rot3(gtsam::Rot3::Retract(*v));
}
gtsam::Vector3* Rot3_OriginLocalCoordinates(const gtsam::Rot3* g) {
    return new gtsam::Vector3(gtsam::Rot3::LocalCoordinates(*g));
}
gtsam::Rot3* Rot3_OriginRetractH(const gtsam::Vector3* v, gtsam::Matrix* H) {
    return new gtsam::Rot3(gtsam::Rot3::Retract(*v, *H));
}
gtsam::Vector3* Rot3_OriginLocalCoordinatesH(const gtsam::Rot3* g,
                                             gtsam::Matrix* H) {
    return new gtsam::Vector3(gtsam::Rot3::LocalCoordinates(*g, *H));
}

gtsam::Vector3* Rot3_Logmap(const gtsam::Rot3* p) {
    return new gtsam::Vector3(gtsam::Rot3::Logmap(*p));
}
gtsam::Vector3* Rot3_LogmapH(const gtsam::Rot3* p, gtsam::Matrix* H) {
    return new gtsam::Vector3(gtsam::Rot3::Logmap(*p, *H));
}
gtsam::Rot3* Rot3_Expmap(const gtsam::Vector3* xi) {
    return new gtsam::Rot3(gtsam::Rot3::Expmap(*xi));
}
gtsam::Rot3* Rot3_ExpmapH(const gtsam::Vector3* xi, gtsam::Matrix* H) {
    return new gtsam::Rot3(gtsam::Rot3::Expmap(*xi, *H));
}
gtsam::Unit3* Rot3_rotateUnit3(const gtsam::Rot3* r, const gtsam::Unit3* p) {
    return new gtsam::Unit3(r->rotate(*p));
}
gtsam::Point3* Rot3_rotatePoint3(const gtsam::Rot3* r, const gtsam::Point3* p) {
    return new gtsam::Point3(r->rotate(*p));
}
gtsam::Unit3* Rot3_rotateUnit3H(const gtsam::Rot3* r,   //
                                const gtsam::Unit3* p,  //
                                gtsam::Matrix* H1,      //
                                gtsam::Matrix* H2) {    //
    return new gtsam::Unit3(r->rotate(*p, *H1, *H2));
}
gtsam::Point3* Rot3_rotatePoint3H(const gtsam::Rot3* r,    //
                                  const gtsam::Point3* p,  //
                                  gtsam::Matrix* H1,       //
                                  gtsam::Matrix* H2) {     //
    return new gtsam::Point3(r->rotate(*p, *H1, *H2));
}
gtsam::Unit3* Rot3_unrotateUnit3(const gtsam::Rot3* r, const gtsam::Unit3* p) {
    return new gtsam::Unit3(r->unrotate(*p));
}
gtsam::Point3* Rot3_unrotatePoint3(const gtsam::Rot3* r,      //
                                   const gtsam::Point3* p) {  //
    return new gtsam::Point3(r->unrotate(*p));
}
gtsam::Unit3* Rot3_unrotateUnit3H(const gtsam::Rot3* r,   //
                                  const gtsam::Unit3* p,  //
                                  gtsam::Matrix* H1,      //
                                  gtsam::Matrix* H2) {    //
    return new gtsam::Unit3(r->unrotate(*p, *H1, *H2));
}
gtsam::Point3* Rot3_unrotatePoint3H(const gtsam::Rot3* r,    //
                                    const gtsam::Point3* p,  //
                                    gtsam::Matrix* H1,       //
                                    gtsam::Matrix* H2) {     //
    return new gtsam::Point3(r->unrotate(*p, *H1, *H2));
}
bool Rot3_check_group_invariants(const gtsam::Rot3* a,  //
                                 const gtsam::Rot3* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Rot3_check_manifold_invariants(const gtsam::Rot3* a,    //
                                    const gtsam::Rot3* b) {  //
    return gtsam::check_manifold_invariants(*a, *b);
}
struct AxisAngle {
    void* first;
    double second;
};
AxisAngle Rot3_axisAngle(const gtsam::Rot3* r) {
    std::pair<gtsam::Unit3, double> a = r->axisAngle();
    return {new gtsam::Unit3(a.first), a.second};
}
gtsam::Rot3* Rot3_ClosestTo(const gtsam::Matrix3* M) {
    return new gtsam::Rot3(gtsam::Rot3::ClosestTo(*M));
}
gtsam::Rot3* Rot3_Rx(double t) {
    return new gtsam::Rot3(gtsam::Rot3::Rx(t));
}
gtsam::Rot3* Rot3_Ry(double t) {
    return new gtsam::Rot3(gtsam::Rot3::Ry(t));
}
gtsam::Rot3* Rot3_Rz(double t) {
    return new gtsam::Rot3(gtsam::Rot3::Rz(t));
}
gtsam::Rot3* Rot3_RzRyRx(double x, double y, double z) {
    return new gtsam::Rot3(gtsam::Rot3::RzRyRx(x, y, z));
}
gtsam::Rot3* Rot3_RzRyRxH(double x, double y, double z,
                          gtsam::Matrix* H1,  //
                          gtsam::Matrix* H2,  //
                          gtsam::Matrix* H3) {
    return new gtsam::Rot3(gtsam::Rot3::RzRyRx(x, y, z, *H1, *H2, *H3));
}
gtsam::Rot3* Rot3_RzRyRxVector3(const gtsam::Vector3* xyz) {
    return new gtsam::Rot3(gtsam::Rot3::RzRyRx(*xyz));
}
gtsam::Rot3* Rot3_RzRyRxVector3H(const gtsam::Vector3* xyz, gtsam::Matrix* H) {
    return new gtsam::Rot3(gtsam::Rot3::RzRyRx(*xyz, *H));
}
gtsam::Rot3* Rot3_normalized(const gtsam::Rot3* r) {
    return new gtsam::Rot3(r->normalized());
}
double Rot3_roll(const gtsam::Rot3* r) {
    return r->roll();
}
double Rot3_pitch(const gtsam::Rot3* r) {
    return r->pitch();
}
double Rot3_yaw(const gtsam::Rot3* r) {
    return r->yaw();
}
double Rot3_rollH(const gtsam::Rot3* r, gtsam::Matrix* H) {
    return r->roll(*H);
}
double Rot3_pitchH(const gtsam::Rot3* r, gtsam::Matrix* H) {
    return r->pitch(*H);
}
double Rot3_yawH(const gtsam::Rot3* r, gtsam::Matrix* H) {
    return r->yaw(*H);
}
gtsam::Vector3* Rot3_xyz(const gtsam::Rot3* r) {
    return new gtsam::Vector3(r->xyz());
}
gtsam::Vector3* Rot3_ypr(const gtsam::Rot3* r) {
    return new gtsam::Vector3(r->ypr());
}
gtsam::Vector3* Rot3_rpy(const gtsam::Rot3* r) {
    return new gtsam::Vector3(r->rpy());
}
gtsam::Vector3* Rot3_xyzH(const gtsam::Rot3* r, gtsam::Matrix* H) {
    return new gtsam::Vector3(r->xyz(*H));
}
gtsam::Vector3* Rot3_yprH(const gtsam::Rot3* r, gtsam::Matrix* H) {
    return new gtsam::Vector3(r->ypr(*H));
}
gtsam::Vector3* Rot3_rpyH(const gtsam::Rot3* r, gtsam::Matrix* H) {
    return new gtsam::Vector3(r->rpy(*H));
}
PtrPair Rot3_RQ(const gtsam::Matrix3* A) {
    std::pair<gtsam::Matrix3, gtsam::Vector3> p = gtsam::RQ(*A);
    return {new gtsam::Matrix3(p.first), new gtsam::Vector3(p.second)};
}
PtrPair Rot3_RQH(const gtsam::Matrix3* A, gtsam::Matrix* H) {
    std::pair<gtsam::Matrix3, gtsam::Vector3> p = gtsam::RQ(*A, *H);
    return {new gtsam::Matrix3(p.first), new gtsam::Vector3(p.second)};
}
gtsam::Quaternion* Rot3_toQuaternion(const gtsam::Rot3* r) {
    return new gtsam::Quaternion(r->toQuaternion());
}
}