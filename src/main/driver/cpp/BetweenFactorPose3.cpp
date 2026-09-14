#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>

extern "C" {
/**
 * @param measured is copied (BetweenFactor.measured_ is not a reference type)
 */
std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>* BetweenFactorPose3(
    gtsam::Key key1,               //
    gtsam::Key key2,               //
    const gtsam::Pose3* measured,  //
    const gtsam::SharedNoiseModel* model) {
    return new std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>(  //
        new gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, *measured, *model));
}
/** @param p shared_ptr* */
double BetweenFactorPose3_error(                  //
    const gtsam::BetweenFactor<gtsam::Pose3>* p,  //
    const gtsam::Values* v) {                     //
    return p->error(*v);
}

gtsam::Vector6* BetweenFactorPose3_evaluateError(  //
    const gtsam::BetweenFactor<gtsam::Pose3>* f,   //
    const gtsam::Pose3* R1,                        //
    const gtsam::Pose3* R2) {                      //
    return new gtsam::Vector6(f->evaluateError(*R1, *R2));
}
gtsam::Vector6* BetweenFactorPose3_evaluateErrorH(  //
    const gtsam::BetweenFactor<gtsam::Pose3>* f,    //
    const gtsam::Pose3* R1,                         //
    const gtsam::Pose3* R2,                         //
    gtsam::Matrix* H1,                              //
    gtsam::Matrix* H2) {                            //
    return new gtsam::Vector6(f->evaluateError(*R1, *R2, *H1, *H2));
}
}