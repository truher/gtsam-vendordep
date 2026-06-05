#include <gtsam/slam/BetweenFactor.h>

extern "C" {
std::shared_ptr<gtsam::BetweenFactor<double>>* BetweenFactorDouble(
    gtsam::Key key1,                         //
    gtsam::Key key2,                         //
    double measured,                         //
    const gtsam::SharedNoiseModel* model) {  //
    return new std::shared_ptr<gtsam::BetweenFactor<double>>(
        new gtsam::BetweenFactor<double>(key1, key2, measured, *model));
}
gtsam::Vector1* BetweenFactorDouble_evaluateError(  //
    const gtsam::BetweenFactor<double>* f,          //
    double R1,                                      //
    double R2) {                                    //
    return new gtsam::Vector1(f->evaluateError(R1, R2));
}
gtsam::Vector1* BetweenFactorDouble_evaluateErrorH(  //
    const gtsam::BetweenFactor<double>* f,           //
    double R1,                                       //
    double R2,                                       //
    gtsam::Matrix* H1,                               //
    gtsam::Matrix* H2) {                             //
    return new gtsam::Vector1(f->evaluateError(R1, R2, *H1, *H2));
}
}