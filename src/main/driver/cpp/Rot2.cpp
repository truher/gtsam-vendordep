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
gtsam::Point2* Rot2_unrotate(const gtsam::Rot2* r, const gtsam::Point2* p) {
    return new gtsam::Point2(r->unrotate((*p)));
}
gtsam::Point2* Rot2_unrotateH(const gtsam::Rot2* r,    //
                              const gtsam::Point2* p,  //
                              gtsam::Matrix* H1,       //
                              gtsam::Matrix* H2) {     //
    return new gtsam::Point2(r->unrotate(*p, *H1, *H2));
}
gtsam::Rot2* Rot2_fromCosSin(double c, double s) {
    return new gtsam::Rot2(gtsam::Rot2::fromCosSin(c, s));
}
gtsam::Rot2* Rot2_fromDegrees(double degrees) {
    return new gtsam::Rot2(gtsam::Rot2::fromDegrees(degrees));
}
gtsam::Rot2* Rot2_atan2(double y, double x) {
    return new gtsam::Rot2(gtsam::Rot2::atan2(y, x));
}
gtsam::Rot2* Rot2_relativeBearing(const gtsam::Point2* d) {
    return new gtsam::Rot2(gtsam::Rot2::relativeBearing(*d));
}
gtsam::Rot2* Rot2_relativeBearingH(const gtsam::Point2* d, gtsam::Matrix* H) {
    return new gtsam::Rot2(gtsam::Rot2::relativeBearing(*d, *H));
}
gtsam::Point2* Rot2_unit(const gtsam::Rot2* r) {
    return new gtsam::Point2(r->unit());
}
gtsam::Rot2* Rot2_inverse(const gtsam::Rot2* r) {
    return new gtsam::Rot2(r->inverse());
}
gtsam::Rot2* Rot2_inverseH(const gtsam::Rot2* r, gtsam::Matrix* H) {
    return new gtsam::Rot2(r->inverse(*H));
}
gtsam::Matrix* Rot2_AdjointMap(const gtsam::Rot2* r) {
    return new gtsam::Matrix(r->AdjointMap());
}
// gtsam::Rot2* Rot2_expmap(const gtsam::Rot2* r, const gtsam::Vector1* v) {
//     return new gtsam::Rot2(r->expmap(*v));
// }
gtsam::Rot2* Rot2_expmapH(const gtsam::Rot2* r,     //
                          const gtsam::Vector1* v,  //
                          gtsam::Matrix* H1,        //
                          gtsam::Matrix* H2) {      //
    return new gtsam::Rot2(r->expmap(*v, *H1, *H2));
}
// gtsam::Vector1* Rot2_logmap(const gtsam::Rot2* r,  //
//                             const gtsam::Rot2* g) {
//     return new gtsam::Vector1(r->logmap(*g));
// }
gtsam::Vector1* Rot2_logmapH(const gtsam::Rot2* r,  //
                             const gtsam::Rot2* g,  //
                             gtsam::Matrix* H1,     //
                             gtsam::Matrix* H2) {   //
    return new gtsam::Vector1(r->logmap(*g, *H1, *H2));
}

gtsam::Rot2* Rot2_OriginRetract(const gtsam::Vector1* v) {
    return new gtsam::Rot2(gtsam::Rot2::Retract(*v));
}
gtsam::Rot2* Rot2_OriginRetractH(const gtsam::Vector1* v, gtsam::Matrix* H) {
    return new gtsam::Rot2(gtsam::Rot2::Retract(*v, *H));
}
gtsam::Matrix2* Rot2_transpose(const gtsam::Rot2* r) {
    return new gtsam::Matrix2(r->transpose());
}
gtsam::Rot2* Rot2_between(const gtsam::Rot2* r,    //
                          const gtsam::Rot2* g) {  //
    return new gtsam::Rot2(r->between(*g));
}
gtsam::Rot2* Rot2_betweenH(const gtsam::Rot2* r,  //
                           const gtsam::Rot2* g,  //
                           gtsam::Matrix* H1,     //
                           gtsam::Matrix* H2) {   //
    return new gtsam::Rot2(r->between(*g, *H1, *H2));
}
gtsam::Rot2* Rot2_retract(const gtsam::Rot2* r, const gtsam::Vector1* v) {
    return new gtsam::Rot2(r->retract(*v));
}
gtsam::Rot2* Rot2_retractH(const gtsam::Rot2* r,     //
                           const gtsam::Vector1* v,  //
                           gtsam::Matrix* H1,        //
                           gtsam::Matrix* H2) {      //
    return new gtsam::Rot2(r->retract(*v, *H1, *H2));
}
gtsam::Vector1* Rot2_localCoordinates(const gtsam::Rot2* r,
                                      const gtsam::Rot2* g) {
    return new gtsam::Vector1(r->localCoordinates(*g));
}
gtsam::Vector1* Rot2_localCoordinatesH(const gtsam::Rot2* r,  //
                                       const gtsam::Rot2* g,  //
                                       gtsam::Matrix* H1,     //
                                       gtsam::Matrix* H2) {   //
    return new gtsam::Vector1(r->localCoordinates(*g, *H1, *H2));
}
gtsam::Vector1* Rot2_Logmap(const gtsam::Rot2* p) {
    return new gtsam::Vector1(gtsam::Rot2::Logmap(*p));
}
gtsam::Vector1* Rot2_LogmapH(const gtsam::Rot2* p, gtsam::Matrix* H) {
    return new gtsam::Vector1(gtsam::Rot2::Logmap(*p, *H));
}
gtsam::Rot2* Rot2_Expmap(const gtsam::Vector1* xi) {
    return new gtsam::Rot2(gtsam::Rot2::Expmap(*xi));
}
gtsam::Rot2* Rot2_ExpmapH(const gtsam::Vector1* xi, gtsam::Matrix* H) {
    return new gtsam::Rot2(gtsam::Rot2::Expmap(*xi, *H));
}
bool Rot2_check_group_invariants(const gtsam::Rot2* a, const gtsam::Rot2* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Rot2_check_manifold_invariants(const gtsam::Rot2* a,
                                    const gtsam::Rot2* b) {
    return gtsam::check_manifold_invariants(*a, *b);
}
void Rot2_print(const gtsam::Rot2* r) {
    r->print();
}
}