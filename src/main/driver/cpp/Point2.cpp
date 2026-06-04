#include <gtsam/geometry/Point2.h>

extern "C" {
gtsam::Point2* Point2(double x, double y) {
    return new gtsam::Point2(x, y);
}
void Point2_delete(gtsam::Point2* p) {
    delete p;
}
gtsam::Point2* Point2_plus(const gtsam::Point2* v,        //
                           const gtsam::Point2* other) {  //
    return new gtsam::Point2((*v) + (*other));
}
gtsam::Point2* Point2_minus(const gtsam::Point2* v,  //
                            const gtsam::Point2* other) {
    return new gtsam::Point2((*v) - (*other));
}
gtsam::Point2* Point2_times(const gtsam::Point2* p, double a) {
    return new gtsam::Point2((*p) * a);
}
double Point2_x(const gtsam::Point2* p) {
    return p->x();
}
double Point2_y(const gtsam::Point2* p) {
    return p->y();
}
double Point2_norm2(const gtsam::Point2* p) {
    return gtsam::norm2(*p);
}
double Point2_norm2H(const gtsam::Point2* p, gtsam::Matrix* H) {
    return gtsam::norm2(*p, *H);
}
double Point2_distance2(const gtsam::Point2* p, const gtsam::Point2* q) {
    return gtsam::distance2(*p, *q);
}
double Point2_norm(const gtsam::Point2* p) {
    return p->norm();
}
double Point2_distance2H(const gtsam::Point2* p,  //
                         const gtsam::Point2* q,  //
                         gtsam::Matrix* H1,       //
                         gtsam::Matrix* H2) {     //
    return gtsam::distance2(*p, *q, *H1, *H2);
}
bool Point2_check_group_invariants(const gtsam::Point2* a,  //
                                   const gtsam::Point2* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Point2_check_manifold_invariants(const gtsam::Point2* a,    //
                                      const gtsam::Point2* b) {  //
    return gtsam::check_manifold_invariants(*a, *b);
}
gtsam::Point2* Point2_normalized(const gtsam::Point2* p) {
    return p->normalized();
}

gtsam::Vector2* Point2_Logmap(const gtsam::Point2* p) {
    return new gtsam::Vector2(gtsam::traits<gtsam::Point2>::Logmap(*p));
}
gtsam::Point2* Point2_Expmap(const gtsam::Vector2* v) {
    return new gtsam::Point2(gtsam::traits<gtsam::Point2>::Expmap(*v));
}
gtsam::Vector2* Point2_LogmapH(const gtsam::Point2* p,  //
                               gtsam::Matrix* H) {      //
    return new gtsam::Vector2(gtsam::traits<gtsam::Point2>::Logmap(*p, *H));
}
gtsam::Point2* Point2_ExpmapH(const gtsam::Vector2* v,  //
                              gtsam::Matrix* H) {       //
    return new gtsam::Point2(gtsam::traits<gtsam::Point2>::Expmap(*v, *H));
}
gtsam::Point2* Point2_OriginRetract(const gtsam::Vector2* v) {
    return new gtsam::Point2(gtsam::Point2::Retract(*v));
}
gtsam::Vector2* Point2_OriginLocalCoordinates(const gtsam::Point2* g) {
    return new gtsam::Vector2(gtsam::Point2::LocalCoordinates(*g));
}
gtsam::Point2* Point2_OriginRetractH(const gtsam::Vector2* v,  //
                                     gtsam::Matrix* H) {       //
    return new gtsam::Point2(gtsam::Point2::Retract(*v, *H));
}
gtsam::Vector2* Point2_OriginLocalCoordinatesH(const gtsam::Point2* g,  //
                                               gtsam::Matrix* H) {      //
    return new gtsam::Vector2(gtsam::Point2::LocalCoordinates(*g, *H));
}

gtsam::Point2* Point2_compose(const gtsam::Point2* a,  //
                              const gtsam::Point2* b) {
    return new gtsam::Point2((*a) * (*b));
}
gtsam::Point2* Point2_composeH(const gtsam::Point2* a,  //
                               const gtsam::Point2* b,  //
                               gtsam::Matrix* H1,       //
                               gtsam::Matrix* H2) {     //
    return new gtsam::Point2(a->compose(*b, *H1, *H2));
}
gtsam::Point2* Point2_between(const gtsam::Point2* r,    //
                              const gtsam::Point2* g) {  //
    return new gtsam::Point2(r->between(*g));
}
gtsam::Point2* Point2_betweenH(const gtsam::Point2* r,  //
                               const gtsam::Point2* g,  //
                               gtsam::Matrix* H1,       //
                               gtsam::Matrix* H2) {     //
    return new gtsam::Point2(r->between(*g, *H1, *H2));
}

gtsam::Point2* Point2_inverse(const gtsam::Point2* r) {
    return new gtsam::Point2(r->inverse());
}
gtsam::Point2* Point2_inverseH(const gtsam::Point2* p,  //
                               gtsam::Matrix* H) {      //
    return new gtsam::Point2(p->inverse(*H));
}
}