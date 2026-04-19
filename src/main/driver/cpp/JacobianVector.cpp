#include <gtsam/nonlinear/CustomFactor.h>

extern "C" {
// this is never deleted, it is owned by the caller
// void JacobianVector_delete(gtsam::JacobianVector* p) {
//     delete p;
// }
void JacobianVector_insert(gtsam::JacobianVector* v,  //
                           int i,                     //
                           gtsam::Matrix* m) {
    v->insert(v->begin() + i, *m);
}
void JacobianVector_insertMatrix3(gtsam::JacobianVector* v,  //
                                  int i,                     //
                                  gtsam::Matrix3* m) {
    v->insert(v->begin() + i, *m);
}
}