#include <gtsam/nonlinear/FixedLagSmoother.h>

extern "C" {
//
// RESULT
//
void Result_delete(gtsam::FixedLagSmoother::Result* p) {
    delete p;
}
//
// KEY TIMESTAMP MAP
//
gtsam::FixedLagSmoother::KeyTimestampMap* KeyTimestampMap() {
    return new gtsam::FixedLagSmoother::KeyTimestampMap();
}
void KeyTimestampMap_delete(gtsam::FixedLagSmoother::KeyTimestampMap* p) {
    delete p;
}
// TODO: maybe move this?
void KeyTimestampMap_put(gtsam::FixedLagSmoother::KeyTimestampMap* p,
                         gtsam::Key k, double v) {
    (*p)[k] = v;
}
void KeyTimestampMap_clear(gtsam::FixedLagSmoother::KeyTimestampMap* p) {
    p->clear();
}
//
// FIXED LAG SMOOTHER
//
gtsam::FixedLagSmoother::Result* FixedLagSmoother_update(  //
    gtsam::FixedLagSmoother* p,                            //
    const gtsam::NonlinearFactorGraph* newFactors,              //
    const gtsam::Values* newTheta,                              //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps) {
    return new gtsam::FixedLagSmoother::Result(
        p->update(*newFactors, *newTheta, *timestamps));
}
gtsam::FixedLagSmoother::Result* FixedLagSmoother_updateFactorIndices(  //
    gtsam::FixedLagSmoother* p,                                         //
    const gtsam::NonlinearFactorGraph* newFactors,                           //
    const gtsam::Values* newTheta,                                           //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps,              //
    const gtsam::FactorIndices* indices) {
    return new gtsam::FixedLagSmoother::Result(
        p->update(*newFactors, *newTheta, *timestamps, *indices));
}
gtsam::Values* FixedLagSmoother_calculateEstimate(
    const gtsam::FixedLagSmoother* p) {
    return new gtsam::Values(p->calculateEstimate());
}
}