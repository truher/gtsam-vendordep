#define _USE_MATH_DEFINES
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/ProjectionFactor.h>

#include <cmath>

extern "C" {
std::shared_ptr<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                               gtsam::Cal3DS2>>*
GenericProjectionFactorCal3DS2(                //
    const gtsam::Point2* measured,             //
    const gtsam::SharedNoiseModel* model,      //
    const gtsam::Key poseKey,                  //
    const gtsam::Key pointKey,                 //
    const std::shared_ptr<gtsam::Cal3DS2>* K,  //
    const bool throwCheirality,                //
    const bool verboseCheirality) {
    return new std::shared_ptr<gtsam::GenericProjectionFactor<
        gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(  //
        new gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                           gtsam::Cal3DS2>(  //
            *measured, *model, poseKey, pointKey, *K, throwCheirality,
            verboseCheirality));
}
gtsam::Vector2* GenericProjectionFactorCal3DS2_evaluateError(
    gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3,
                                   gtsam::Cal3DS2>* p,  //
    const gtsam::Pose3* pose,                           //
    const gtsam::Point3* point,                         //
    gtsam::Matrix* H1,                                  //
    gtsam::Matrix* H2) {
    return new gtsam::Vector2(p->evaluateError(*pose, *point, H1, H2));
}
std::shared_ptr<gtsam::GenericProjectionFactor2<gtsam::Cal3DS2>>*
GenericProjectionFactor2Cal3DS2(const gtsam::Key poseKey,              //
                                const gtsam::Key calibKey,             //
                                const gtsam::Point3* landmark,         //
                                const gtsam::Point2* measured,         //
                                const gtsam::SharedNoiseModel* model,  //
                                const bool verboseCheirality) {
    return new std::shared_ptr<gtsam::GenericProjectionFactor2<gtsam::Cal3DS2>>(
        new gtsam::GenericProjectionFactor2<gtsam::Cal3DS2>(
            poseKey, calibKey, *landmark, *measured, *model,
            verboseCheirality));
}
gtsam::Vector2* GenericProjectionFactor2Cal3DS2_evaluateError(
    gtsam::GenericProjectionFactor2<gtsam::Cal3DS2>* p,  //
    const gtsam::Pose3* pose3,                           //
    const gtsam::Cal3DS2* calib,                         //
    gtsam::Matrix* Dpose,                                //
    gtsam::Matrix* Dcal) {
    return new gtsam::Vector2(p->evaluateError(*pose3, *calib, Dpose, Dcal));
}
std::shared_ptr<gtsam::GenericProjectionFactor3<gtsam::Cal3DS2>>*
GenericProjectionFactor3Cal3DS2(const gtsam::Key poseKey,                  //
                                const gtsam::Point3* landmark,             //
                                const gtsam::Point2* measured,             //
                                const std::shared_ptr<gtsam::Cal3DS2>* K,  //
                                const gtsam::SharedNoiseModel* model,      //
                                const bool verboseCheirality) {
    return new std::shared_ptr<gtsam::GenericProjectionFactor3<gtsam::Cal3DS2>>(
        new gtsam::GenericProjectionFactor3<gtsam::Cal3DS2>(
            poseKey, *landmark, *measured, *K, *model, verboseCheirality));
}
gtsam::Vector2* GenericProjectionFactor3Cal3DS2_evaluateError(
    gtsam::GenericProjectionFactor3<gtsam::Cal3DS2>* p,  //
    const gtsam::Pose3* pose3,                           //
    gtsam::Matrix* Dpose) {
    return new gtsam::Vector2(p->evaluateError(*pose3, Dpose));
}
}