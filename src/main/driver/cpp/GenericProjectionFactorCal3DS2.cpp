#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/ProjectionFactor.h>

extern "C" {
std::shared_ptr<                     //
    gtsam::GenericProjectionFactor<  //
        gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>*
GenericProjectionFactorCal3DS2(                                 //
    const gtsam::Point2* measured,                              //
    const gtsam::SharedNoiseModel* model,                       //
    gtsam::Key poseKey,                                         //
    gtsam::Key pointKey,                                        //
    const std::shared_ptr<gtsam::Cal3DS2>* K,                   //
    const gtsam::Pose3* bTc) {                                  //
    return new std::shared_ptr<gtsam::GenericProjectionFactor<  //
        gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(          //
        new gtsam::GenericProjectionFactor(                     //
            *measured, *model, poseKey, pointKey, *K, *bTc));
}
}