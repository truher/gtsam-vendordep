#include <gtsam/base/types.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PlanarProjectionFactor.h>

extern "C" {
/**
 * Returns a pointer to a heap-allocated shared pointer to the factor, so that
 * Java can manage the lifespan of the shared pointer.
 *
 * TODO: actually do that.
 *
 * @param key  pass-by-value since it's a primitive typedef
 */
std::shared_ptr<gtsam::PlanarProjectionFactor1>* PlanarProjectionFactor1(
    const gtsam::Key poseKey,       //
    const gtsam::Point3* landmark,  //
    const gtsam::Point2* measured,  //
    const gtsam::Pose3* bTc,        //
    const gtsam::Cal3DS2* calib,    //
    const gtsam::SharedNoiseModel* model) {
    // Because we want to pass this back as a shared pointer,
    // we need to create it here, and pass a pointer to it.
    return new std::shared_ptr<gtsam::PlanarProjectionFactor1>(  //
        new gtsam::PlanarProjectionFactor1(                      //
            poseKey, *landmark, *measured, *bTc, *calib, *model));
}

/** OptionalMatrixType is typedef Matrix*
 *
 * TODO: maybe this should be Vector instead of Vector2
 */
gtsam::Vector2* PlanarProjectionFactor1_evaluateError(
    gtsam::PlanarProjectionFactor1* p,  //
    const gtsam::Pose2* wTb,                             //
    gtsam::Matrix* HwTb) {
    return new gtsam::Vector2(p->evaluateError(*wTb, *HwTb));
}
}