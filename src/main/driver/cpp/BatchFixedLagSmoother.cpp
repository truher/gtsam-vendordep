#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

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
gtsam::FixedLagSmoother::Result* BatchFixedLagSmoother_update(  //
    gtsam::BatchFixedLagSmoother* p,                            //
    const gtsam::NonlinearFactorGraph* newFactors,              //
    const gtsam::Values* newTheta,                              //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps) {
    // this can throw, which breaks the Java side, so catch here.
    try {
        return new gtsam::FixedLagSmoother::Result(
            p->update(*newFactors, *newTheta, *timestamps));
    } catch (const gtsam::IndeterminateSystemException e) {
        std::cout << "caught IndeterminateSystemException: " << e.what()
                  << std::endl;
        return nullptr;
    }
}
gtsam::FixedLagSmoother::Result* BatchFixedLagSmoother_updateFactorIndices(  //
    gtsam::BatchFixedLagSmoother* p,                                         //
    const gtsam::NonlinearFactorGraph* newFactors,                           //
    const gtsam::Values* newTheta,                                           //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps,              //
    const gtsam::FactorIndices* indices) {
    // this can throw, which breaks the Java side, so catch here.
    try {
        return new gtsam::FixedLagSmoother::Result(
            p->update(*newFactors, *newTheta, *timestamps, *indices));
    } catch (const gtsam::IndeterminateSystemException e) {
        std::cout << "caught IndeterminateSystemException: " << e.what()
                  << std::endl;
        return nullptr;
    }
}
gtsam::Values* BatchFixedLagSmoother_calculateEstimate(
    const gtsam::BatchFixedLagSmoother* p) {
    return new gtsam::Values(p->calculateEstimate());
}
gtsam::Point2* BatchFixedLagSmoother_calculateEstimatePoint2(
    const gtsam::BatchFixedLagSmoother* p, gtsam::Key key) {
    return new gtsam::Point2(p->calculateEstimate<gtsam::Point2>(key));
}
gtsam::NonlinearFactorGraph* BatchFixedLagSmoother_getFactors(
    const gtsam::BatchFixedLagSmoother* p) {
    return new gtsam::NonlinearFactorGraph(p->getFactors());
}
}