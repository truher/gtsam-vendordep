#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

extern "C" {
gtsam::BatchFixedLagSmoother* BatchFixedLagSmoother(double lag) {
    return new gtsam::BatchFixedLagSmoother(lag);
}
gtsam::BatchFixedLagSmoother* BatchFixedLagSmoother2(
    double lag,                               //
    gtsam::LevenbergMarquardtParams* params,  //
    bool consistent) {
    return new gtsam::BatchFixedLagSmoother(lag, *params, consistent);
}
void BatchFixedLagSmoother_delete(gtsam::BatchFixedLagSmoother* p) {
    delete p;
}
gtsam::Point2* BatchFixedLagSmoother_calculateEstimatePoint2(
    const gtsam::BatchFixedLagSmoother* p, gtsam::Key key) {
    return new gtsam::Point2(p->calculateEstimate<gtsam::Point2>(key));
}
gtsam::Pose2* BatchFixedLagSmoother_calculateEstimatePose2(
    const gtsam::BatchFixedLagSmoother* p, gtsam::Key key) {
    return new gtsam::Pose2(p->calculateEstimate<gtsam::Pose2>(key));
}
gtsam::NonlinearFactorGraph* BatchFixedLagSmoother_getFactors(
    const gtsam::BatchFixedLagSmoother* p) {
    return new gtsam::NonlinearFactorGraph(p->getFactors());
}
}