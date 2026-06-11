#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

extern "C" {
gtsam::IncrementalFixedLagSmoother* IncrementalFixedLagSmoother(
    double lag,  //
    gtsam::ISAM2Params* params) {
    return new gtsam::IncrementalFixedLagSmoother(lag, *params);
}
void IncrementalFixedLagSmoother_delete(      //
    gtsam::IncrementalFixedLagSmoother* p) {  //
    delete p;
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
gtsam::Values* IncrementalFixedLagSmoother_getLinearizationPoint(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::Values(p->getLinearizationPoint());
}
gtsam::ISAM2* IncrementalFixedLagSmoother_getISAM2(
    const gtsam::IncrementalFixedLagSmoother* p) {
    return new gtsam::ISAM2(p->getISAM2());
}
}