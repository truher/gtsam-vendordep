#include <gtsam/geometry/Point3.h>

extern "C" {
gtsam::Point3* Point3(double x, double y, double z) {
    return new gtsam::Point3(x, y, z);
}
void Point3_delete(gtsam::Point3* obj) {
    delete obj;
}
gtsam::Point3* Point3_plus(const gtsam::Point3* v, const gtsam::Point3* other) {
    return new gtsam::Point3((*v) + (*other));
}
gtsam::Point3* Point3_minus(const gtsam::Point3* v,
                            const gtsam::Point3* other) {
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
}