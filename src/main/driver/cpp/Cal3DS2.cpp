#include <gtsam/base/Vector.h>
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
gtsam::Vector9* Cal3DS2_localCoordinates(const gtsam::Cal3DS2* a,
                                        const gtsam::Cal3DS2* b) {
    return new gtsam::Vector9(a->localCoordinates(*b));
}
gtsam::Cal3DS2* Cal3DS2_retract(const gtsam::Cal3DS2* p,
                                const gtsam::Vector9* v) {
    return new gtsam::Cal3DS2(p->retract(*v));
}
}