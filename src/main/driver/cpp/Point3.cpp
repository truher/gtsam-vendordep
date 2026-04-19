#include <gtsam/geometry/Point3.h>

extern "C" {
gtsam::Point3* Point3(double x, double y, double z) {
    return new gtsam::Point3(x, y, z);
}
void Point3_delete(gtsam::Point3* obj) {
    delete obj;
}
void Point3_print(gtsam::Point3* p) {
    std::cout << *p << std::endl;
}
}