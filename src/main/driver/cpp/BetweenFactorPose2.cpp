#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>

extern "C" {
/**
 * @param measured is copied (BetweenFactor.measured_ is not a reference type)
 */
std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>* BetweenFactorPose2(
    gtsam::Key key1,               //
    gtsam::Key key2,               //
    const gtsam::Pose2* measured,  //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>(  //
        new gtsam::BetweenFactor<gtsam::Pose2>(key1, key2, *measured, *model));
}
/** @param p shared_ptr* */
double BetweenFactorPose2_error(                  //
    const gtsam::BetweenFactor<gtsam::Pose2>* p,  //
    const gtsam::Values* v) {                     //
    return p->error(*v);
}
}