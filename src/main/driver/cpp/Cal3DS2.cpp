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
gtsam::Point2* Cal3DS2_uncalibrate(const gtsam::Cal3DS2* c,
                                   const gtsam::Point2* p) {
    return new gtsam::Point2(c->uncalibrate(*p));
}
gtsam::Point2* Cal3DS2_uncalibrateH(const gtsam::Cal3DS2* c,  //
                                    const gtsam::Point2* p,   //
                                    gtsam::Matrix* H1,        //
                                    gtsam::Matrix* H2) {      //
    return new gtsam::Point2(c->uncalibrate(*p, *H1, *H2));
}
gtsam::Point2* Cal3DS2_calibrate(const gtsam::Cal3DS2* c,
                                 const gtsam::Point2* p) {
    return new gtsam::Point2(c->calibrate(*p));
}

gtsam::Point2* Cal3DS2_calibrateH(const gtsam::Cal3DS2* c,  //
                                  const gtsam::Point2* p,   //
                                  gtsam::Matrix* H1,        //
                                  gtsam::Matrix* H2) {      //
    return new gtsam::Point2(c->calibrate(*p, *H1, *H2));
}
gtsam::Matrix* Cal3DS2_D2d_calibration(const gtsam::Cal3DS2* k,
                                       const gtsam::Point2* p) {
    return new gtsam::Matrix(k->D2d_calibration(*p));
}
gtsam::Matrix* Cal3DS2_D2d_intrinsic(const gtsam::Cal3DS2* k,
                                     const gtsam::Point2* p) {
    return new gtsam::Matrix(k->D2d_intrinsic(*p));
}
gtsam::Vector4* Cal3DS2_k(const gtsam::Cal3DS2* k) {
    return new gtsam::Vector4(k->k());
}
gtsam::Matrix3* Cal3DS2_K(const gtsam::Cal3DS2* k) {
    return new gtsam::Matrix3(k->K());
}
}