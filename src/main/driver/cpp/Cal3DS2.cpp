#include <gtsam/geometry/Cal3DS2.h>

extern "C" {
gtsam::Cal3DS2* Cal3DS2(double fx, double fy,            //
                        double s, double u0, double v0,  //
                        double k1, double k2,            //
                        double p1, double p2,            //
                        double tol) {
    return new gtsam::Cal3DS2(fx, fy, s, u0, v0, k1, k2, p1, p2, tol);
}
void Cal3DS2_delete(gtsam::Cal3DS2* p) {
    delete p;
}
}