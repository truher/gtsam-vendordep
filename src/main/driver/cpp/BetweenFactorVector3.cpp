#include <gtsam/base/Vector.h>
#include <gtsam/slam/BetweenFactor.h>

extern "C" {
std::shared_ptr<gtsam::BetweenFactor<gtsam::Vector3>>* BetweenFactorVector3(
    gtsam::Key key1,                                                   //
    gtsam::Key key2,                                                   //
    const gtsam::Vector3* measured,                                    //
    const gtsam::SharedNoiseModel* model) {                            //
    return new std::shared_ptr<gtsam::BetweenFactor<gtsam::Vector3>>(  //
        new gtsam::BetweenFactor<gtsam::Vector3>(                      //
            key1, key2, *measured, *model));
}
gtsam::Vector3* BetweenFactorVector3_evaluateError(  //
    const gtsam::BetweenFactor<gtsam::Vector3>* f,   //
    const gtsam::Vector3* R1,                        //
    const gtsam::Vector3* R2) {                      //
    return new gtsam::Vector3(f->evaluateError(*R1, *R2));
}
gtsam::Vector3* BetweenFactorVector3_evaluateErrorH(  //
    const gtsam::BetweenFactor<gtsam::Vector3>* f,    //
    const gtsam::Vector3* R1,                         //
    const gtsam::Vector3* R2,                         //
    gtsam::Matrix* H1,                                //
    gtsam::Matrix* H2) {                              //
    return new gtsam::Vector3(f->evaluateError(*R1, *R2, *H1, *H2));
}
}