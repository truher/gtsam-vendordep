#include <gtsam/base/Lie.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>

extern "C" {
////////////////////////////
// Vector
gtsam::Vector* Vector(int size) {
    // Java users certainly expect zero initialization!
    gtsam::Vector* v = new gtsam::Vector(size);
    v->setZero();
    return v;
}
void Vector_delete(gtsam::Vector* p) {
    delete p;
}
gtsam::Vector* Vector_fromTangentVector(gtsam::Pose2::TangentVector* v) {
    return new gtsam::Vector(*v);
}
gtsam::Vector* Vector_fromVector2(gtsam::Vector2* v) {
    return new gtsam::Vector(*v);
}
gtsam::Vector* Vector_fromVector3(gtsam::Vector3* v) {
    return new gtsam::Vector(*v);
}
// TODO: there must be some vectorspace trait i can use here?
gtsam::Vector* Vector_Local(gtsam::Vector* a, gtsam::Vector* b) {
    return new gtsam::Vector((*b) - (*a));
}
void Vector_set(gtsam::Vector* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector* Vector_minus(const gtsam::Vector* v,
                            const gtsam::Vector* other) {
    return new gtsam::Vector((*v) - (*other));
}
gtsam::Vector2* Vector2_minus(const gtsam::Vector2* v,
                              const gtsam::Vector2* other) {
    return new gtsam::Vector2((*v) - (*other));
}
gtsam::Vector3* Vector3_minus(const gtsam::Vector3* v,
                              const gtsam::Vector3* other) {
    return new gtsam::Vector3((*v) - (*other));
}
gtsam::Vector* Vector_plus(const gtsam::Vector* v, const gtsam::Vector* other) {
    return new gtsam::Vector((*v) + (*other));
}
gtsam::Vector2* Vector2_plus(const gtsam::Vector2* v,
                             const gtsam::Vector2* other) {
    return new gtsam::Vector2((*v) + (*other));
}
gtsam::Vector3* Vector3_plus(const gtsam::Vector3* v,
                             const gtsam::Vector3* other) {
    // std::cout << "v " << *v << std::endl;
    // std::cout << "other " << *other << std::endl;
    return new gtsam::Vector3((*v) + (*other));
}
gtsam::Vector* Vector_times(const gtsam::Vector* v, double a) {
    return new gtsam::Vector((*v) * a);
}
double Vector_at(const gtsam::Vector* v, int i) {
    return (*v)(i);
}
int Vector_rows(const gtsam::Vector* v) {
    return v->rows();
}
bool Vector_equals(const gtsam::Vector* a, const gtsam::Vector* b, double tol) {
    return gtsam::equal_with_abs_tol(*a, *b, tol);
}
bool Vector3_equals(const gtsam::Vector3* a, const gtsam::Vector3* b,
                    double tol) {
    return gtsam::equal_with_abs_tol(*a, *b, tol);
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