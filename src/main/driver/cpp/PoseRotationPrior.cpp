#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PoseRotationPrior.h>

extern "C" {
std::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose2>>* PoseRotationPriorPose2(
    const gtsam::Key key,      //
    const gtsam::Pose2* pose,  //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose2>>(
        new gtsam::PoseRotationPrior<gtsam::Pose2>(key, *pose, *model));
}
void PoseRotationPriorPose2_delete(
    std::shared_ptr<gtsam::PoseRotationPrior<gtsam::Pose2>>* obj) {
    delete obj;
}
}