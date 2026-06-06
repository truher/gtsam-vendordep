#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>

/**
 * prior is copied, ok to delete
 * TODO: make separate files for each type (like BetweenFactor)
 */
extern "C" {
std::shared_ptr<gtsam::PriorFactor<double>>* PriorFactorDouble(  //
    const gtsam::Key key,                                        //
    double prior,                                                //
    const std::shared_ptr<gtsam::noiseModel::Base>* model) {     //
    return new std::shared_ptr<gtsam::PriorFactor<double>>(
        new gtsam::PriorFactor<double>(key, prior, *model));
}
std::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>* PriorFactorPose2(  //
    const gtsam::Key key,                                             //
    const gtsam::Pose2* prior,                                        //
    const gtsam::SharedNoiseModel* model) {                           //
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>(
        new gtsam::PriorFactor<gtsam::Pose2>(key, *prior, *model));
}
std::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>* PriorFactorPose3(  //
    const gtsam::Key key,                                             //
    const gtsam::Pose3* prior,                                        //
    const gtsam::SharedNoiseModel* model) {                           //
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>(
        new gtsam::PriorFactor<gtsam::Pose3>(key, *prior, *model));
}
std::shared_ptr<gtsam::PriorFactor<gtsam::Vector>>* PriorFactorVector(  //
    const gtsam::Key key,                                                 //
    const gtsam::Vector* prior,                                          //
    const gtsam::SharedNoiseModel* model) {                               //
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Vector>>(
        new gtsam::PriorFactor<gtsam::Vector>(key, *prior, *model));
}
std::shared_ptr<gtsam::PriorFactor<gtsam::Vector3>>* PriorFactorVector3(  //
    const gtsam::Key key,                                                 //
    const gtsam::Vector3* prior,                                          //
    const gtsam::SharedNoiseModel* model) {                               //
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Vector3>>(
        new gtsam::PriorFactor<gtsam::Vector3>(key, *prior, *model));
}
std::shared_ptr<gtsam::PriorFactor<gtsam::Cal3DS2>>* PriorFactorCal3DS2(  //
    const gtsam::Key key,                                                 //
    const gtsam::Cal3DS2* prior,                                          //
    const gtsam::SharedNoiseModel* model) {                               //
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Cal3DS2>>(
        new gtsam::PriorFactor<gtsam::Cal3DS2>(key, *prior, *model));
}
}