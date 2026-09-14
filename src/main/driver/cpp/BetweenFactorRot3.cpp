#include <gtsam/geometry/Rot3.h>
#include <gtsam/slam/BetweenFactor.h>

extern "C" {
std::shared_ptr<gtsam::BetweenFactor<gtsam::Rot3>>* BetweenFactorRot3(
    gtsam::Key key1,                                                //
    gtsam::Key key2,                                                //
    const gtsam::Rot3* measured,                                    //
    const gtsam::SharedNoiseModel* model) {                         //
    return new std::shared_ptr<gtsam::BetweenFactor<gtsam::Rot3>>(  //
        new gtsam::BetweenFactor<gtsam::Rot3>(key1, key2, *measured, *model));
}
gtsam::Vector3* BetweenFactorRot3_evaluateError(  //
    const gtsam::BetweenFactor<gtsam::Rot3>* f,   //
    const gtsam::Rot3* R1,                        //
    const gtsam::Rot3* R2) {                      //
    return new gtsam::Vector3(f->evaluateError(*R1, *R2));
}
gtsam::Vector3* BetweenFactorRot3_evaluateErrorH(  //
    const gtsam::BetweenFactor<gtsam::Rot3>* f,    //
    const gtsam::Rot3* R1,                         //
    const gtsam::Rot3* R2,                         //
    gtsam::Matrix* H1,                             //
    gtsam::Matrix* H2) {                           //
    return new gtsam::Vector3(f->evaluateError(*R1, *R2, *H1, *H2));
}
}