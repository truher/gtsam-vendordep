#define _USE_MATH_DEFINES
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/GeneralSFMFactor.h>

#include <cmath>

extern "C" {
std::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3DS2>>*
GeneralSFMFactor2Cal3DS2(                  //
    const gtsam::Point2* measured,         //
    const gtsam::SharedNoiseModel* model,  //
    const gtsam::Key poseKey,              //
    const gtsam::Key landmarkKey,          //
    const gtsam::Key calibKey) {
    return new std::shared_ptr<gtsam::GeneralSFMFactor2<gtsam::Cal3DS2>>(  //
        new gtsam::GeneralSFMFactor2<gtsam::Cal3DS2>(                      //
            *measured, *model, poseKey, landmarkKey, calibKey));
}
gtsam::Vector2* GeneralSFMFactor2Cal3DS2_evaluateError(
    gtsam::GeneralSFMFactor2<gtsam::Cal3DS2>* p,  //
    const gtsam::Pose3* pose3,                    //
    const gtsam::Point3* point,                   //
    const gtsam::Cal3DS2* calib,                  //
    gtsam::Matrix* H1,                            //
    gtsam::Matrix* H2,                            //
    gtsam::Matrix* H3) {
    return new gtsam::Vector2(
        p->evaluateError(*pose3, *point, *calib, H1, H2, H3));
}
}