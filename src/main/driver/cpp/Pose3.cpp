#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

extern "C" {
/**
 * Pose3 constructor uses the implicit copy constructors for each member
 * (rotation, translation), so the arguments here can be freed.
 */
gtsam::Pose3* Pose3(const gtsam::Rot3* r, const gtsam::Point3* t) {
    return new gtsam::Pose3(*r, *t);
}
void Pose3_delete(gtsam::Pose3* p) {
    delete p;
}
gtsam::Pose3* Pose3_Pose2(const gtsam::Pose2* p) {
    return new gtsam::Pose3(*p);
}
gtsam::Pose3* Pose3_compose(const gtsam::Pose3* p,  //
                            const gtsam::Pose3* p2) {
    return new gtsam::Pose3(p->compose(*p2));
}
gtsam::Pose3* Pose3_composeH(const gtsam::Pose3* p,   //
                             const gtsam::Pose3* p2,  //
                             gtsam::Matrix* H1,       //
                             gtsam::Matrix* H2) {     //
    return new gtsam::Pose3(p->compose(*p2, *H1, *H2));
}
gtsam::Pose3* Pose3_between(const gtsam::Pose3* p,  //
                            const gtsam::Pose3* p2) {
    return new gtsam::Pose3(p->between(*p2));
}
gtsam::Pose3* Pose3_betweenH(const gtsam::Pose3* p,  //
                             const gtsam::Pose3* p2,
                             gtsam::Matrix* H1,    //
                             gtsam::Matrix* H2) {  //
    return new gtsam::Pose3(p->between(*p2, *H1, *H2));
}
gtsam::Pose3* Pose3_retract(const gtsam::Pose3* p,  //
                            const gtsam::Vector6* v) {
    return new gtsam::Pose3(p->retract(*v));
}
gtsam::Pose3* Pose3_retractH(const gtsam::Pose3* p,    //
                             const gtsam::Vector6* v,  //
                             gtsam::Matrix* H1,        //
                             gtsam::Matrix* H2) {      //
    return new gtsam::Pose3(p->retract(*v, *H1, *H2));
}
gtsam::Vector6* Pose3_localCoordinates(const gtsam::Pose3* a,
                                       const gtsam::Pose3* b) {
    return new gtsam::Vector6(a->localCoordinates(*b));
}
gtsam::Vector6* Pose3_localCoordinatesH(const gtsam::Pose3* a,  //
                                        const gtsam::Pose3* b,  //
                                        gtsam::Matrix* H1,      //
                                        gtsam::Matrix* H2) {    //
    return new gtsam::Vector6(a->localCoordinates(*b, *H1, *H2));
}
gtsam::Pose3* Pose3_inverse(const gtsam::Pose3* p) {
    return new gtsam::Pose3(p->inverse());
}
gtsam::Pose3* Pose3_inverseH(const gtsam::Pose3* p, gtsam::Matrix* H) {
    return new gtsam::Pose3(p->inverse(*H));
}
/** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
gtsam::Matrix* Pose3_AdjointMap(const gtsam::Pose3* p) {
    return new gtsam::Matrix(p->AdjointMap());
}
gtsam::Pose3* Pose3_expmapH(const gtsam::Pose3* p,    //
                            const gtsam::Vector6* v,  //
                            gtsam::Matrix* H1,        //
                            gtsam::Matrix* H2) {
    return new gtsam::Pose3(p->expmap(*v, *H1, *H2));
}
gtsam::Vector6* Pose3_logmapH(const gtsam::Pose3* p,  //
                              const gtsam::Pose3* g,  //
                              gtsam::Matrix* H1,      //
                              gtsam::Matrix* H2) {
    return new gtsam::Vector6(p->logmap(*g, *H1, *H2));
}
gtsam::Vector6* Pose3_logmap(const gtsam::Pose3* r, const gtsam::Pose3* g) {
    return new gtsam::Vector6(r->logmap(*g));
}
gtsam::Pose3* Pose3_expmap(const gtsam::Pose3* r, const gtsam::Vector6* v) {
    return new gtsam::Pose3(r->expmap(*v));
}
gtsam::Vector6* Pose3_LogmapH(const gtsam::Pose3* p, gtsam::Matrix* H) {
    return new gtsam::Vector6(gtsam::Pose3::Logmap(*p, *H));
}
gtsam::Pose3* Pose3_ExpmapH(const gtsam::Vector6* xi, gtsam::Matrix* H) {
    return new gtsam::Pose3(gtsam::Pose3::Expmap(*xi, *H));
}
gtsam::Matrix* Pose3_matrix(const gtsam::Pose3* p) {
    // 4x4 matrix
    return new gtsam::Matrix(p->matrix());
}
bool Pose3_check_group_invariants(const gtsam::Pose3* a,  //
                                  const gtsam::Pose3* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Pose3_check_manifold_invariants(const gtsam::Pose3* a,    //
                                     const gtsam::Pose3* b) {  //
    return gtsam::check_manifold_invariants(*a, *b);
}
gtsam::Unit3* Pose3_bearingPoint3(const gtsam::Pose3* p, gtsam::Point3* pt) {
    return new gtsam::Unit3(p->bearing(*pt));
}
gtsam::Unit3* Pose3_bearingPoint3H(const gtsam::Pose3* p,    //
                                   const gtsam::Point3* pt,  //
                                   gtsam::Matrix* H1,        //
                                   gtsam::Matrix* H2) {      //
    return new gtsam::Unit3(p->bearing(*pt, *H1, *H2));
}
gtsam::Unit3* Pose3_bearingPose3(const gtsam::Pose3* p, gtsam::Pose3* p2) {
    return new gtsam::Unit3(p->bearing(*p2));
}
gtsam::Unit3* Pose3_bearingPose3H(const gtsam::Pose3* p,   //
                                  const gtsam::Pose3* p2,  //
                                  gtsam::Matrix* H1,       //
                                  gtsam::Matrix* H2) {     //
    return new gtsam::Unit3(p->bearing(*p2, *H1, *H2));
}
double Pose3_rangePoint3(const gtsam::Pose3* p, gtsam::Point3* pt) {
    return p->range(*pt);
}
double Pose3_rangePoint3H(const gtsam::Pose3* p,  //
                          gtsam::Point3* pt,      //
                          gtsam::Matrix* H1,      //
                          gtsam::Matrix* H2) {    //
    return p->range(*pt, *H1, *H2);
}
double Pose3_rangePose3(const gtsam::Pose3* p,  //
                        gtsam::Pose3* p2) {     //
    return p->range(*p2);
}
double Pose3_rangePose3H(const gtsam::Pose3* p,  //
                         gtsam::Pose3* p2,       //
                         gtsam::Matrix* H1,      //
                         gtsam::Matrix* H2) {    //
    return p->range(*p2, *H1, *H2);
}
gtsam::Vector6* Pose3_logmap_default(const gtsam::Pose3* a,    //
                                     const gtsam::Pose3* b) {  //
    return new gtsam::Vector6(gtsam::logmap_default(*a, *b));
}
gtsam::Pose3* Pose3_expmap_default(const gtsam::Pose3* p,      //
                                   const gtsam::Vector6* d) {  //
    return new gtsam::Pose3(gtsam::expmap_default(*p, *d));
}
gtsam::Vector6* Pose3_Adjoint(const gtsam::Pose3* p,       //
                              const gtsam::Vector6* xi) {  //
    return new gtsam::Vector6(p->Adjoint(*xi));
}
gtsam::Vector6* Pose3_AdjointH(const gtsam::Pose3* p,     //
                               const gtsam::Vector6* xi,  //
                               gtsam::Matrix* H1,         //
                               gtsam::Matrix* H2) {       //
    return new gtsam::Vector6(p->Adjoint(*xi, *H1, *H2));
}
gtsam::Rot3* Pose3_rotation(const gtsam::Pose3* p) {
    return new gtsam::Rot3(p->rotation());
}
gtsam::Rot3* Pose3_rotationH(const gtsam::Pose3* p,  //
                             gtsam::Matrix* H) {     //
    return new gtsam::Rot3(p->rotation(*H));
}
gtsam::Point3* Pose3_translation(const gtsam::Pose3* p) {
    return new gtsam::Point3(p->translation());
}
gtsam::Point3* Pose3_translationH(const gtsam::Pose3* p,  //
                                  gtsam::Matrix* H) {     //
    return new gtsam::Point3(p->translation(*H));
}
gtsam::Point3* Pose3_transformTo(const gtsam::Pose3* p,         //
                                 const gtsam::Point3* point) {  //
    return new gtsam::Point3(p->transformTo(*point));
}
gtsam::Point3* Pose3_transformToH(const gtsam::Pose3* p,       //
                                  const gtsam::Point3* point,  //
                                  gtsam::Matrix* Dpose,        //
                                  gtsam::Matrix* Dpoint) {     //
    return new gtsam::Point3(p->transformTo(*point, *Dpose, *Dpoint));
}
gtsam::Point3* Pose3_transformFrom(const gtsam::Pose3* p,         //
                                   const gtsam::Point3* point) {  //
    return new gtsam::Point3(p->transformFrom(*point));
}
gtsam::Point3* Pose3_transformFromH(const gtsam::Pose3* p,       //
                                    const gtsam::Point3* point,  //
                                    gtsam::Matrix* Dpose,        //
                                    gtsam::Matrix* Dpoint) {     //
    return new gtsam::Point3(p->transformFrom(*point, *Dpose, *Dpoint));
}
gtsam::Pose3* Pose3_interpolate(const gtsam::Pose3* X,  //
                                const gtsam::Pose3* Y,  //
                                double t) {             //
    return new gtsam::Pose3(gtsam::interpolate(*X, *Y, t));
}
gtsam::Pose3* Pose3_interpolateH(const gtsam::Pose3* X,  //
                                 const gtsam::Pose3* Y,  //
                                 double t,               //
                                 gtsam::Matrix* H1,      //
                                 gtsam::Matrix* H2,      //
                                 gtsam::Matrix* H3) {    //
    return new gtsam::Pose3(gtsam::interpolate(*X, *Y, t, *H1, *H2, *H3));
}

gtsam::Pose3* Pose3_interpolateRt(const gtsam::Pose3* X,  //
                                  const gtsam::Pose3* Y,  //
                                  double t) {             //
    return new gtsam::Pose3(X->interpolateRt(*Y, t));
}
gtsam::Pose3* Pose3_interpolateRtH(const gtsam::Pose3* X,  //
                                   const gtsam::Pose3* Y,  //
                                   double t,               //
                                   gtsam::Matrix* H1,      //
                                   gtsam::Matrix* H2,      //
                                   gtsam::Matrix* H3) {    //
    return new gtsam::Pose3(X->interpolateRt(*Y, t, *H1, *H2, *H3));
}
gtsam::Pose3* Pose3_transformPoseFrom(const gtsam::Pose3* a,    //
                                      const gtsam::Pose3* b) {  //
    return new gtsam::Pose3(a->transformPoseFrom(*b));
}
gtsam::Pose3* Pose3_transformPoseFromH(const gtsam::Pose3* a,  //
                                       const gtsam::Pose3* b,
                                       gtsam::Matrix* H1,    //
                                       gtsam::Matrix* H2) {  //
    return new gtsam::Pose3(a->transformPoseFrom(*b, *H1, *H2));
}
gtsam::Pose3* Pose3_transformPoseTo(const gtsam::Pose3* a,    //
                                    const gtsam::Pose3* b) {  //
    return new gtsam::Pose3(a->transformPoseTo(*b));
}
gtsam::Pose3* Pose3_transformPoseToH(const gtsam::Pose3* a,  //
                                     const gtsam::Pose3* b,  //
                                     gtsam::Matrix* H1,      //
                                     gtsam::Matrix* H2) {    //
    return new gtsam::Pose3(a->transformPoseTo(*b, *H1, *H2));
}
gtsam::Pose3* Pose3_Create(const gtsam::Rot3* R,  //
                           const gtsam::Point3* t) {
    return new gtsam::Pose3(gtsam::Pose3::Create(*R, *t));
}
gtsam::Pose3* Pose3_CreateH(const gtsam::Rot3* R,    //
                           const gtsam::Point3* t,  //
                           gtsam::Matrix* H1,       //
                           gtsam::Matrix* H2) {
    return new gtsam::Pose3(gtsam::Pose3::Create(*R, *t, *H1, *H2));
}
}