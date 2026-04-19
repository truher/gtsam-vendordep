#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

extern "C" {
void Result_delete(gtsam::FixedLagSmoother::Result* p) {
    delete p;
}
gtsam::BatchFixedLagSmoother* BatchFixedLagSmoother(double lag) {
    return new gtsam::BatchFixedLagSmoother(lag);
}
void BatchFixedLagSmoother_delete(gtsam::BatchFixedLagSmoother* p) {
    delete p;
}
// Result is a struct with some counters
// typedef std::map<Key, double> KeyTimestampMap;
gtsam::FixedLagSmoother::Result* BatchFixedLagSmoother_update(  //
    gtsam::BatchFixedLagSmoother* p,                            //
    const gtsam::NonlinearFactorGraph* newFactors,              //
    const gtsam::Values* newTheta,                              //
    const gtsam::FixedLagSmoother::KeyTimestampMap* timestamps) {
    return new gtsam::FixedLagSmoother::Result(
        p->update(*newFactors, *newTheta, *timestamps));
}
gtsam::Values* BatchFixedLagSmoother_calculateEstimate(
    const gtsam::BatchFixedLagSmoother* p) {
    return new gtsam::Values(p->calculateEstimate());
}
gtsam::NonlinearFactorGraph* BatchFixedLagSmoother_getFactors(
    const gtsam::BatchFixedLagSmoother* p) {
    return new gtsam::NonlinearFactorGraph(p->getFactors());
}

////////////////////////////

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
}