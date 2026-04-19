#include <gtsam/geometry/Rot3.h>

extern "C" {
/// Construct from a rotation matrix, as doubles in *row-major* order !!!
gtsam::Rot3* Rot3(                       //
    double R11, double R12, double R13,  //
    double R21, double R22, double R23,  //
    double R31, double R32, double R33) {
    return new gtsam::Rot3(R11, R12, R13,  //
                           R21, R22, R23,  //
                           R31, R32, R33);
}
void Rot3_delete(gtsam::Rot3* p) {
    delete p;
}
}