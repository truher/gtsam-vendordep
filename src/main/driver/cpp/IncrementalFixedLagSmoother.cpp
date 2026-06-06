#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

extern "C" {
gtsam::IncrementalFixedLagSmoother* IncrementalFixedLagSmoother2(
    double lag,  //
    gtsam::ISAM2Params* params) {
    return new gtsam::IncrementalFixedLagSmoother(lag, *params);
}
void IncrementalFixedLagSmoother_delete(      //
    gtsam::IncrementalFixedLagSmoother* p) {  //
    delete p;
}
gtsam::FixedLagSmoother::Result* IncrementalFixedLagSmoother_update(  //
    gtsam::IncrementalFixedLagSmoother* p,                            //
    const gtsam::NonlinearFactorGraph* newFactors,                    //
    const gtsam::Values* newTheta,                                    //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps) {
    return new gtsam::FixedLagSmoother::Result(
        p->update(*newFactors, *newTheta, *timestamps));
}
gtsam::FixedLagSmoother::Result*
IncrementalFixedLagSmoother_updateFactorIndices(                 //
    gtsam::IncrementalFixedLagSmoother* p,                       //
    const gtsam::NonlinearFactorGraph* newFactors,               //
    const gtsam::Values* newTheta,                               //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps,  //
    const gtsam::FactorIndices* indices) {
    return new gtsam::FixedLagSmoother::Result(
        p->update(*newFactors, *newTheta, *timestamps, *indices));
}
gtsam::Values* IncrementalFixedLagSmoother_calculateEstimate(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::Values(p->calculateEstimate());
}
gtsam::Point2* IncrementalFixedLagSmoother_calculateEstimatePoint2(
    const gtsam::IncrementalFixedLagSmoother* p, gtsam::Key key) {
    return new gtsam::Point2(p->calculateEstimate<gtsam::Point2>(key));
}
gtsam::NonlinearFactorGraph* IncrementalFixedLagSmoother_getFactors(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::NonlinearFactorGraph(p->getFactors());
}
gtsam::ISAM2Result* IncrementalFixedLagSmoother_getISAM2Result(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::ISAM2Result(p->getISAM2Result());
}
)
gtsam::Values* IncrementalFixedLagSmoother_getLinearizationPoint(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::Values(p->getLinearizationPoint());
}
gtsam::ISAM2* IncrementalFixedLagSmoother_getISAM2(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::ISAM2(p->getISAM2());
}
}