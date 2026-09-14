#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/BetweenFactor.h>

extern "C" {
std::shared_ptr<gtsam::BetweenFactor<gtsam::Point2>>* BetweenFactorPoint2(
    gtsam::Key key1,                                                //
    gtsam::Key key2,                                                //
    const gtsam::Point2* measured,                                    //
    const gtsam::SharedNoiseModel* model) {                         //
    return new std::shared_ptr<gtsam::BetweenFactor<gtsam::Point2>>(  //
        new gtsam::BetweenFactor<gtsam::Point2>(key1, key2, *measured, *model));
}
gtsam::Vector2* BetweenFactorPoint2_evaluateError(  //
    const gtsam::BetweenFactor<gtsam::Point2>* f,   //
    const gtsam::Point2* R1,                        //
    const gtsam::Point2* R2) {                      //
    return new gtsam::Vector2(f->evaluateError(*R1, *R2));
}
gtsam::Vector2* BetweenFactorPoint2_evaluateErrorH(  //
    const gtsam::BetweenFactor<gtsam::Point2>* f,    //
    const gtsam::Point2* R1,                         //
    const gtsam::Point2* R2,                         //
    gtsam::Matrix* H1,                             //
    gtsam::Matrix* H2) {                           //
    return new gtsam::Vector2(f->evaluateError(*R1, *R2, *H1, *H2));
}
}