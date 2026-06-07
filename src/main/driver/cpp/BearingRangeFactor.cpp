#include <gtsam/sam/BearingRangeFactor.h>

extern "C" {
std::shared_ptr<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>*
BearingRangeFactorPose2Point2(  //
    gtsam::Key key1,
    gtsam::Key key2,             //
    const gtsam::Rot2* bearing,  //
    double range,                //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::BearingRangeFactor<Pose2, Point2>>(
        new gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
            key1, key2, *bearing, range, *model));
}
}