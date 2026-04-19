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
}