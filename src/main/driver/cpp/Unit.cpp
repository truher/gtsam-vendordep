#include <gtsam/geometry/Unit3.h>

extern "C" {
gtsam::Unit3* Unit3(double x, double y, double z) {
    return new gtsam::Unit3(x, y, z);
}
gtsam::Unit3* Unit3Point3(const gtsam::Point3* p) {
    return new gtsam::Unit3(*p);
}
void Unit3_delete(gtsam::Unit3* obj) {
    delete obj;
}
gtsam::Point3* Unit3_point3(const gtsam::Unit3* p) {
    return new gtsam::Point3(p->point3());
}
gtsam::Point3* Unit3_point3H(const gtsam::Unit3* p,  //
                             gtsam::Matrix* H) {     //
    return new gtsam::Point3(p->point3(*H));
}
gtsam::Vector2* Unit3_errorVector(const gtsam::Unit3* p,    //
                                  const gtsam::Unit3* q) {  //
    return new gtsam::Vector2(p->errorVector(*q));
}
gtsam::Vector2* Unit3_errorVectorH(const gtsam::Unit3* p,  //
                                   const gtsam::Unit3* q,  //
                                   gtsam::Matrix* H1,      //
                                   gtsam::Matrix* H2) {    //
    return new gtsam::Vector2(p->errorVector(*q, *H1, *H2));
}
gtsam::Unit3* Unit3_retract(const gtsam::Unit3* p,      //
                            const gtsam::Vector2* v) {  //
    return new gtsam::Unit3(p->retract(*v));
}
gtsam::Unit3* Unit3_retractH(const gtsam::Unit3* p,    //
                             const gtsam::Vector2* v,  //
                             gtsam::Matrix* H) {       //
    return new gtsam::Unit3(p->retract(*v, *H));
}
gtsam::Vector2* Unit3_localCoordinates(const gtsam::Unit3* p,    //
                                       const gtsam::Unit3* s) {  //
    return new gtsam::Vector2(p->localCoordinates(*s));
}
double Unit3_dot(const gtsam::Unit3* p,    //
                 const gtsam::Unit3* q) {  //
    return p->dot(*q);
}
double Unit3_dotH(const gtsam::Unit3* p,  //
                  const gtsam::Unit3* q,  //
                  gtsam::Matrix* H1,      //
                  gtsam::Matrix* H2) {    //
    return p->dot(*q, *H1, *H2);
}
double Unit3_distance(const gtsam::Unit3* p,    //
                      const gtsam::Unit3* q) {  //
    return p->distance(*q);
}
double Unit3_distanceH(const gtsam::Unit3* p,  //
                       const gtsam::Unit3* q,  //
                       gtsam::Matrix* H) {     //
    return p->distance(*q, *H);
}
gtsam::Matrix* Unit3_basis(const gtsam::Unit3* p) {  //
    return new gtsam::Matrix(p->basis());
}
gtsam::Matrix* Unit3_basisH(const gtsam::Unit3* p,  //
                            gtsam::Matrix* H) {     //
    return new gtsam::Matrix(p->basis(*H));
}
gtsam::Vector3* Unit3_unitVector(const gtsam::Unit3* p) {
    return new gtsam::Vector3(p->unitVector());
}
gtsam::Unit3* Unit3_FromPoint(const gtsam::Point3* p) {
    return new gtsam::Unit3(gtsam::Unit3::FromPoint3(*p));
}
gtsam::Unit3* Unit3_FromPointH(const gtsam::Point3* p,  //
                               gtsam::Matrix* H) {
    return new gtsam::Unit3(gtsam::Unit3::FromPoint3(*p, *H));
}
gtsam::Unit3* Unit3_crossUnit3Unit3(  //
    const gtsam::Unit3* p,           //
    const gtsam::Unit3* q) {         //
    return new gtsam::Unit3(gtsam::cross(*p, *q));
}
gtsam::Unit3* Unit3_crossUnit3Unit3H(  //
    const gtsam::Unit3* p,             //
    const gtsam::Unit3* q,             //
    gtsam::Matrix* H1,                 //
    gtsam::Matrix* H2) {               //
    return new gtsam::Unit3(gtsam::cross(*p, *q, *H1, *H2));
}
gtsam::Point3* Unit3_crossUnit3Point3H(  //
    const gtsam::Unit3* p,               //
    const gtsam::Point3* q,              //
    gtsam::Matrix* H1,                   //
    gtsam::Matrix* H2) {                 //
    return new gtsam::Point3(gtsam::cross(*p, *q, *H1, *H2));
}
gtsam::Point3* Unit3_crossPoint3Unit3H(  //
    const gtsam::Point3* p,              //
    const gtsam::Unit3* q,               //
    gtsam::Matrix* H1,                   //
    gtsam::Matrix* H2) {                 //
    return new gtsam::Point3(gtsam::cross(*p, *q, *H1, *H2));
}
gtsam::Unit3* Unit3_crossUnit3(const gtsam::Unit3* p,  //
                               const gtsam::Unit3* q) {
    return new gtsam::Unit3(p->cross(*q));
}
gtsam::Point3* Unit3_crossPoint3(const gtsam::Unit3* p,  //
                                 const gtsam::Point3* q) {
    return new gtsam::Point3(p->cross(*q));
}
}