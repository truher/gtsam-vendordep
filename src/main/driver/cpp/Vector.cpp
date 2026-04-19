#include <gtsam/base/Lie.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

extern "C" {
////////////////////////////
// Vector
gtsam::Vector* Vector(int size) {
    return new gtsam::Vector(size);
}
void Vector_delete(gtsam::Vector* p) {
    delete p;
}
gtsam::Vector* Vector_fromTangentVector(gtsam::Pose2::TangentVector* v) {
    return new gtsam::Vector(*v);
}
void Vector_set(gtsam::Vector* v, int i, double val) {
    (*v)(i) = val;
}
double Vector_at(const gtsam::Vector* v, int i) {
    return (*v)(i);
}
int Vector_rows(const gtsam::Vector* v) {
    return v->rows();
}
//////////////////////////
// Vector1
gtsam::Vector1* Vector1(double v0) {
    return new gtsam::Vector1(v0);
}
void Vector1_delete(gtsam::Vector1* p) {
    delete p;
}
////////////////////////////
// Vector2
gtsam::Vector2* Vector2(double v0, double v1) {
    return new gtsam::Vector2(v0, v1);
}
void Vector2_delete(gtsam::Vector2* p) {
    delete p;
}
double Vector2_at(const gtsam::Vector2* v, int i) {
    return (*v)(i);
}
void Vector2_print(const gtsam::Vector2* v) {
    std::cout << *v << std::endl;
}
/////////////////////////
// Vector3
gtsam::Vector3* Vector3(double v0, double v1, double v2) {
    return new gtsam::Vector3(v0, v1, v2);
}
void Vector3_delete(gtsam::Vector3* p) {
    delete p;
}
double Vector3_at(const gtsam::Vector3* v, int i) {
    return (*v)(i);
}
void Vector3_print(const gtsam::Vector3* v) {
    std::cout << *v << std::endl;
}
}