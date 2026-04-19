#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

extern "C" {
gtsam::LevenbergMarquardtOptimizer* LevenbergMarquardtOptimizer(
    const gtsam::NonlinearFactorGraph* graph,
    const gtsam::Values* initialValues) {
    return new gtsam::LevenbergMarquardtOptimizer(*graph, *initialValues);
}
void LevenbergMarquardtOptimizer_delete(gtsam::LevenbergMarquardtOptimizer* p) {
    delete p;
}
gtsam::LevenbergMarquardtOptimizer* LevenbergMarquardtOptimizer3(
    const gtsam::NonlinearFactorGraph* graph,
    const gtsam::Values* initialValues,
    const gtsam::LevenbergMarquardtParams* params) {
    return new gtsam::LevenbergMarquardtOptimizer(*graph,          //
                                                  *initialValues,  //
                                                  *params);
}
const gtsam::Values* LevenbergMarquardtOptimizer_optimize(
    gtsam::LevenbergMarquardtOptimizer* p) {
    return new gtsam::Values(p->optimize());
}
const gtsam::Values* LevenbergMarquardtOptimizer_values(
    gtsam::LevenbergMarquardtOptimizer* p) {
    return new gtsam::Values(p->values());
}
}