#include <gtsam/base/Matrix.h>

extern "C" {
gtsam::Matrix3* Matrix3(                 //
    double R11, double R12, double R13,  //
    double R21, double R22, double R23,  //
    double R31, double R32, double R33) {
    gtsam::Matrix3* m = new gtsam::Matrix3();
    (*m) << R11, R12, R13,  //
        R21, R22, R23,      //
        R31, R32, R33;
    return m;
}
void Matrix3_delete(gtsam::Matrix3* p) {
    delete p;
}
gtsam::Matrix3* Matrix3_unaryMinus(gtsam::Matrix3* m) {
    return new gtsam::Matrix3(-(*m));
}
gtsam::Matrix3* Matrix3_identity() {
    return new gtsam::Matrix3(gtsam::Matrix3::Identity());
}
}