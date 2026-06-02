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

gtsam::Vector* Vector_plus(const gtsam::Vector* v, const gtsam::Vector* other) {
    return new gtsam::Vector((*v) + (*other));
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
//////////////////////////
// Vector1
gtsam::Vector1* Vector1(double v0) {
    return new gtsam::Vector1(v0);
}
void Vector1_delete(gtsam::Vector1* p) {
    delete p;
}

double Vector1_at(const gtsam::Vector1* v, int i) {
    return (*v)(i);
}
void Vector1_set(gtsam::Vector1* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector1* Vector1_plus(const gtsam::Vector1* v,
                             const gtsam::Vector1* other) {
    return new gtsam::Vector1((*v) + (*other));
}
gtsam::Vector1* Vector1_minus(const gtsam::Vector1* v,
                              const gtsam::Vector1* other) {
    return new gtsam::Vector1((*v) - (*other));
}
gtsam::Vector1* Vector1_times(const gtsam::Vector1* v, double a) {
    return new gtsam::Vector1((*v) * a);
}

////////////////////////////
// Vector2
gtsam::Vector2* Vector2() {
    // Java users certainly expect zero initialization!
    gtsam::Vector2* v = new gtsam::Vector2();
    v->setZero();
    return v;
}
void Vector2_delete(gtsam::Vector2* p) {
    delete p;
}
double Vector2_at(const gtsam::Vector2* v, int i) {
    return (*v)(i);
}
void Vector2_set(gtsam::Vector2* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector2* Vector2_plus(const gtsam::Vector2* v,
                             const gtsam::Vector2* other) {
    return new gtsam::Vector2((*v) + (*other));
}
gtsam::Vector2* Vector2_minus(const gtsam::Vector2* v,
                              const gtsam::Vector2* other) {
    return new gtsam::Vector2((*v) - (*other));
}
gtsam::Vector2* Vector2_times(const gtsam::Vector2* v, double a) {
    return new gtsam::Vector2((*v) * a);
}

/////////////////////////
// Vector3
gtsam::Vector3* Vector3() {
    // Java users certainly expect zero initialization!
    gtsam::Vector3* v = new gtsam::Vector3();
    v->setZero();
    return v;
}
void Vector3_delete(gtsam::Vector3* p) {
    delete p;
}
double Vector3_at(const gtsam::Vector3* v, int i) {
    return (*v)(i);
}
void Vector3_set(gtsam::Vector3* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector3* Vector3_plus(const gtsam::Vector3* v,        //
                             const gtsam::Vector3* other) {  //
    return new gtsam::Vector3((*v) + (*other));
}
gtsam::Vector3* Vector3_minus(const gtsam::Vector3* v,        //
                              const gtsam::Vector3* other) {  //
    return new gtsam::Vector3((*v) - (*other));
}
gtsam::Vector3* Vector3_times(const gtsam::Vector3* v, double a) {
    return new gtsam::Vector3((*v) * a);
}
double Vector3_norm(const gtsam::Vector3* v) {
    return v->norm();
}
double Vector3_dot(const gtsam::Vector3* p,    //
                   const gtsam::Vector3* q) {  //
    return p->dot(*q);
}

//////////////////////////
// Vector4
gtsam::Vector4* Vector4() {
    // Java users certainly expect zero initialization!
    gtsam::Vector4* v = new gtsam::Vector4();
    v->setZero();
    return v;
}
void Vector4_delete(gtsam::Vector4* p) {
    delete p;
}
double Vector4_at(const gtsam::Vector4* v, int i) {
    return (*v)(i);
}
void Vector4_set(gtsam::Vector4* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector4* Vector4_plus(const gtsam::Vector4* v,
                             const gtsam::Vector4* other) {
    return new gtsam::Vector4((*v) + (*other));
}
gtsam::Vector4* Vector4_minus(const gtsam::Vector4* v,
                              const gtsam::Vector4* other) {
    return new gtsam::Vector4((*v) - (*other));
}
gtsam::Vector4* Vector4_times(const gtsam::Vector4* v, double a) {
    return new gtsam::Vector4((*v) * a);
}

//////////////////////////
// Vector6
gtsam::Vector6* Vector6() {
    // Java users certainly expect zero initialization!
    gtsam::Vector6* v = new gtsam::Vector6();
    v->setZero();
    return v;
}
void Vector6_delete(gtsam::Vector6* p) {
    delete p;
}
double Vector6_at(const gtsam::Vector6* v, int i) {
    return (*v)(i);
}
void Vector6_set(gtsam::Vector6* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector6* Vector6_plus(const gtsam::Vector6* v,
                             const gtsam::Vector6* other) {
    return new gtsam::Vector6((*v) + (*other));
}
gtsam::Vector6* Vector6_minus(const gtsam::Vector6* v,
                              const gtsam::Vector6* other) {
    return new gtsam::Vector6((*v) - (*other));
}
gtsam::Vector6* Vector6_times(const gtsam::Vector6* v, double a) {
    return new gtsam::Vector6((*v) * a);
}

//////////////////////////
// Vector9
gtsam::Vector9* Vector9() {
    // Java users certainly expect zero initialization!
    gtsam::Vector9* v = new gtsam::Vector9();
    v->setZero();
    return v;
}
void Vector9_delete(gtsam::Vector9* p) {
    delete p;
}
double Vector9_at(const gtsam::Vector9* v, int i) {
    return (*v)(i);
}
void Vector9_set(gtsam::Vector9* v, int i, double val) {
    (*v)(i) = val;
}
gtsam::Vector9* Vector9_plus(const gtsam::Vector9* v,
                             const gtsam::Vector9* other) {
    return new gtsam::Vector9((*v) + (*other));
}
gtsam::Vector9* Vector9_minus(const gtsam::Vector9* v,
                              const gtsam::Vector9* other) {
    return new gtsam::Vector9((*v) - (*other));
}
gtsam::Vector9* Vector9_times(const gtsam::Vector9* v, double a) {
    return new gtsam::Vector9((*v) * a);
}
}