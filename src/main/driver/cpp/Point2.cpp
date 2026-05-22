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
void Point2_print(gtsam::Point2* p) {
    std::cout << *p << std::endl;
}
bool Point2_equals(const gtsam::Point2* a, const gtsam::Point2* b, double tol) {
    return gtsam::equal_with_abs_tol(*a, *b, tol);
}
}