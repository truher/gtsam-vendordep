#include <gtsam/base/Lie.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/Point3.h>

extern "C" {
gtsam::Point3* Point3(double x, double y, double z) {
    return new gtsam::Point3(x, y, z);
}
void Point3_delete(gtsam::Point3* obj) {
    delete obj;
}
gtsam::Point3* Point3_plus(const gtsam::Point3* v,        //
                           const gtsam::Point3* other) {  //
    return new gtsam::Point3((*v) + (*other));
}
gtsam::Point3* Point3_minus(const gtsam::Point3* v,        //
                            const gtsam::Point3* other) {  //
    return new gtsam::Point3((*v) - (*other));
}
gtsam::Point3* Point3_times(const gtsam::Point3* p, double a) {
    return new gtsam::Point3((*p) * a);
}
double Point3_x(const gtsam::Point3* p) {
    return p->x();
}
double Point3_y(const gtsam::Point3* p) {
    return p->y();
}
double Point3_z(const gtsam::Point3* p) {
    return p->z();
}
gtsam::Point3* Point3_crossPoint3Point3(  //
    const gtsam::Point3* p,               //
    const gtsam::Point3* q) {
    return new gtsam::Point3(gtsam::cross(*p, *q));
}
gtsam::Point3* Point3_crossPoint3Point3H(const gtsam::Point3* p,  //
                                         const gtsam::Point3* q,  //
                                         gtsam::Matrix* H1,       //
                                         gtsam::Matrix* H2) {     //
    return new gtsam::Point3(gtsam::cross(*p, *q, *H1, *H2));
}
gtsam::Point3* Point3_cross(const gtsam::Point3* p,    //
                            const gtsam::Point3* q) {  //
    return new gtsam::Point3(p->cross(*q));
}
bool Point3_check_group_invariants(const gtsam::Point3* a,  //
                                   const gtsam::Point3* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Point3_check_manifold_invariants(const gtsam::Point3* a,    //
                                      const gtsam::Point3* b) {  //
    return gtsam::check_manifold_invariants(*a, *b);
}

gtsam::Vector3* Point3_Local(const gtsam::Point3* origin,  //
                             const gtsam::Point3* v) {     //
    return new gtsam::Vector3(gtsam::traits<gtsam::Point3>::Local(*origin, *v));
}
gtsam::Point3* Point3_Retract(const gtsam::Point3* origin,  //
                              const gtsam::Vector3* v) {    //
    return new gtsam::Point3(
        gtsam::traits<gtsam::Point3>::Retract(*origin, *v));
}

gtsam::Vector3* Point3_Logmap(const gtsam::Point3* p) {
    return new gtsam::Vector3(gtsam::traits<gtsam::Point3>::Logmap(*p));
}
gtsam::Point3* Point3_Expmap(const gtsam::Vector3* v) {
    return new gtsam::Point3(gtsam::traits<gtsam::Point3>::Expmap(*v));
}
gtsam::Vector3* Point3_LogmapH(const gtsam::Point3* p,  //
                               gtsam::Matrix* H) {      //
    return new gtsam::Vector3(gtsam::traits<gtsam::Point3>::Logmap(*p, *H));
}
gtsam::Point3* Point3_ExpmapH(const gtsam::Vector3* v,  //
                              gtsam::Matrix* H) {       //
    return new gtsam::Point3(gtsam::traits<gtsam::Point3>::Expmap(*v, *H));
}
gtsam::Point3* Point3_Compose(const gtsam::Point3* a,  //
                              const gtsam::Point3* b) {
    return new gtsam::Point3(gtsam::traits<gtsam::Point3>::Compose(*a, *b));
}
gtsam::Point3* Point3_ComposeH(const gtsam::Point3* a,  //
                               const gtsam::Point3* b,  //
                               gtsam::Matrix* H1,       //
                               gtsam::Matrix* H2) {     //
    return new gtsam::Point3(
        gtsam::traits<gtsam::Point3>::Compose(*a, *b, *H1, *H2));
}
gtsam::Point3* Point3_Between(const gtsam::Point3* r,    //
                              const gtsam::Point3* g) {  //
    return new gtsam::Point3(gtsam::traits<gtsam::Point3>::Between(*r, *g));
}
gtsam::Point3* Point3_BetweenH(const gtsam::Point3* r,  //
                               const gtsam::Point3* g,  //
                               gtsam::Matrix* H1,       //
                               gtsam::Matrix* H2) {     //
    return new gtsam::Point3(
        gtsam::traits<gtsam::Point3>::Between(*r, *g, *H1, *H2));
}
gtsam::Point3* Point3_Inverse(const gtsam::Point3* r) {
    return new gtsam::Point3(gtsam::traits<gtsam::Point3>::Inverse(*r));
}
gtsam::Point3* Point3_InverseH(const gtsam::Point3* p,  //
                               gtsam::Matrix* H) {      //
    return new gtsam::Point3(gtsam::traits<gtsam::Point3>::Inverse(*p, *H));
}
gtsam::Matrix* Point3_AdjointMap(const gtsam::Point3* r) {
    return new gtsam::Matrix(gtsam::traits<gtsam::Point3>::AdjointMap(*r));
}
double Point3_dot(const gtsam::Point3* r,    //
                  const gtsam::Point3* g) {  //
    return r->dot(*g);
}
double Point3_dotPoint3Point3(const gtsam::Point3* p,    //
                              const gtsam::Point3* q) {  //
    return gtsam::dot(*p, *q);
}
double Point3_dotPoint3Point3H(const gtsam::Point3* p,  //
                               const gtsam::Point3* q,
                               gtsam::Matrix* H1,    //
                               gtsam::Matrix* H2) {  //
    return gtsam::dot(*p, *q, *H1, *H2);
}
gtsam::Point3* Point3_normalize(const gtsam::Point3* p) {
    return new gtsam::Point3(gtsam::normalize(*p));
}
gtsam::Point3* Point3_normalizeH(const gtsam::Point3* p,  //
                                 gtsam::Matrix* H) {
    return new gtsam::Point3(gtsam::normalize(*p, *H));
}
double Point3_norm3(const gtsam::Point3* p) {
    return gtsam::norm3(*p);
}
double Point3_norm3H(const gtsam::Point3* p,  //
                     gtsam::Matrix* H) {      //
    return gtsam::norm3(*p, *H);
}
double Point3_norm(const gtsam::Point3* p) {
    return p->norm();
}
double Point3_distance3(const gtsam::Point3* p,    //
                        const gtsam::Point3* q) {  //
    return gtsam::distance3(*p, *q);
}
double Point3_distance3H(const gtsam::Point3* p,  //
                         const gtsam::Point3* q,  //
                         gtsam::Matrix* H1,       //
                         gtsam::Matrix* H2) {     //
    return gtsam::distance3(*p, *q, *H1, *H2);
}
gtsam::Point3* Point3_interpolate(const gtsam::Point3* X,  //
                                  const gtsam::Point3* Y,  //
                                  double t) {              //
    return new gtsam::Point3(gtsam::interpolate(*X, *Y, t));
}
}