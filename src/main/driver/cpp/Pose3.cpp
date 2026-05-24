#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

extern "C" {
/**
 * Pose3 constructor uses the implicit copy constructors for each member
 * (rotation, translation), so the arguments here can be freed.
 */
gtsam::Pose3* Pose3(const gtsam::Rot3* r, const gtsam::Point3* t) {
    return new gtsam::Pose3(*r, *t);
}
void Pose3_delete(gtsam::Pose3* p) {
    delete p;
}
gtsam::Pose3* Pose3_Pose2(const gtsam::Pose2* p) {
    return new gtsam::Pose3(*p);
}
gtsam::Pose3* Pose3_compose(const gtsam::Pose3* p, const gtsam::Pose3* p2) {
    return new gtsam::Pose3(p->compose(*p2));
}
gtsam::Pose3* Pose3_retract(const gtsam::Pose3* p, const gtsam::Vector6* v) {
    return new gtsam::Pose3(p->retract(*v));
}
gtsam::Vector6* Pose3_localCoordinates(const gtsam::Pose3* a,
                                       const gtsam::Pose3* b) {
    return new gtsam::Vector6(a->localCoordinates(*b));
}
gtsam::Pose3* Pose3_inverse(const gtsam::Pose3* p) {
    return new gtsam::Pose3(p->inverse());
}
/** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
gtsam::Matrix* Pose3_AdjointMap(const gtsam::Pose3* p) {
    return new gtsam::Matrix(p->AdjointMap());
}
}