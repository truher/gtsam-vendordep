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
bool Point2_check_group_invariants(const gtsam::Point2* a,  //
                                   const gtsam::Point2* b) {
    return gtsam::check_group_invariants(*a, *b);
}
bool Point2_check_manifold_invariants(const gtsam::Point2* a,    //
                                      const gtsam::Point2* b) {  //
    return gtsam::check_manifold_invariants(*a, *b);
}
}