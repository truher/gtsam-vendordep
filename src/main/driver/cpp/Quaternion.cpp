#include <gtsam/geometry/Quaternion.h>

extern "C" {
gtsam::Quaternion* Quaternion(double w, double x, double y, double z) {
    return new gtsam::Quaternion(w, x, y, z);
}
void Quaternion_delete(gtsam::Quaternion* q) {
    delete q;
}
double Quaternion_w(gtsam::Quaternion* q) {
    return q->w();
}
double Quaternion_x(gtsam::Quaternion* q) {
    return q->x();
}
double Quaternion_y(gtsam::Quaternion* q) {
    return q->y();
}
double Quaternion_z(gtsam::Quaternion* q) {
    return q->z();
}
gtsam::Vector4* Quaternion_coeffs(gtsam::Quaternion* q) {
    return new gtsam::Vector4(q->coeffs());
}
gtsam::Point3* Quaternion_rotate(gtsam::Quaternion* q, gtsam::Point3* p) {
    return new gtsam::Point3((*q) * (*p));
}
}