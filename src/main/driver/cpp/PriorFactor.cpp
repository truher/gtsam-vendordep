#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>

extern "C" {
std::shared_ptr<gtsam::PriorFactor<double>>* PriorFactorDouble(
    const gtsam::Key key, double prior, const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::PriorFactor<double>>(
        new gtsam::PriorFactor<double>(key, prior, *model));
}
void PriorFactorDouble_delete(
    std::shared_ptr<gtsam::PriorFactor<double>>* obj) {
    delete obj;
}
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
void PriorFactorPose2_delete(
    std::shared_ptr<gtsam::PriorFactor<gtsam::Pose2>>* obj) {
    delete obj;
}

std::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>* PriorFactorPose3(
    const gtsam::Key key,       //
    const gtsam::Pose3* prior,  //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>(
        new gtsam::PriorFactor<gtsam::Pose3>(key, *prior, *model));
}
void PriorFactorPose3_delete(
    std::shared_ptr<gtsam::PriorFactor<gtsam::Pose3>>* obj) {
    delete obj;
}

std::shared_ptr<gtsam::PriorFactor<gtsam::Cal3DS2>>* PriorFactorCal3DS2(
    const gtsam::Key key,         //
    const gtsam::Cal3DS2* prior,  //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::PriorFactor<gtsam::Cal3DS2>>(
        new gtsam::PriorFactor<gtsam::Cal3DS2>(key, *prior, *model));
}
void PriorFactorCal3DS2_delete(
    std::shared_ptr<gtsam::PriorFactor<gtsam::Cal3DS2>>* obj) {
    delete obj;
}
}