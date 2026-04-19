#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/PinholePose.h>

extern "C" {
gtsam::PinholeCamera<gtsam::Cal3DS2>* PinholeCameraCal3DS2(
    const gtsam::Pose3* pose, const gtsam::Cal3DS2* K) {
    return new gtsam::PinholeCamera<gtsam::Cal3DS2>(*pose, *K);
}
void PinholeCameraCal3DS2_delete(gtsam::PinholeCamera<gtsam::Cal3DS2>* p) {
    delete p;
}
gtsam::Point2* PinholeCameraCal3DS2_project(
    const gtsam::PinholeCamera<gtsam::Cal3DS2>* camera,
    const gtsam::Point3* pw) {
    return new gtsam::Point2(camera->project(*pw));
}
}