#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>

extern "C" {
/**
 * @param prior is copied, ok to delete
 */
std::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>* PriorFactorPose2(
    const gtsam::Key key,       //
    const gtsam::Pose2* prior,  //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>(
        new gtsam::PriorFactor<gtsam::Pose2>(key, *prior, *model));
}
}