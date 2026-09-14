#include <gtsam/base/Vector.h>
#include <gtsam/slam/BetweenFactor.h>

extern "C" {
std::shared_ptr<gtsam::BetweenFactor<gtsam::Vector>>* BetweenFactorVector(
    gtsam::Key key1,                                                   //
    gtsam::Key key2,                                                   //
    const gtsam::Vector* measured,                                    //
    const gtsam::SharedNoiseModel* model) {                            //
    return new std::shared_ptr<gtsam::BetweenFactor<gtsam::Vector>>(  //
        new gtsam::BetweenFactor<gtsam::Vector>(                      //
            key1, key2, *measured, *model));
}
gtsam::Vector* BetweenFactorVector_evaluateError(  //
    const gtsam::BetweenFactor<gtsam::Vector>* f,   //
    const gtsam::Vector* R1,                        //
    const gtsam::Vector* R2) {                      //
    return new gtsam::Vector(f->evaluateError(*R1, *R2));
}
gtsam::Vector* BetweenFactorVector_evaluateErrorH(  //
    const gtsam::BetweenFactor<gtsam::Vector>* f,    //
    const gtsam::Vector* R1,                         //
    const gtsam::Vector* R2,                         //
    gtsam::Matrix* H1,                                //
    gtsam::Matrix* H2) {                              //
    return new gtsam::Vector(f->evaluateError(*R1, *R2, *H1, *H2));
}
}