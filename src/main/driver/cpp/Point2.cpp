#include <gtsam/geometry/Point2.h>

extern "C" {
gtsam::Point2* Point2(double x, double y) {
    return new gtsam::Point2(x, y);
}
void Point2_delete(gtsam::Point2* p) {
    delete p;
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
}