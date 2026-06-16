#include <gtsam/nonlinear/ISAM2.h>

extern "C" {
void ISAM2_delete(gtsam::ISAM2* p) {
    delete p;
}
gtsam::ISAM2* ISAM2(const gtsam::ISAM2Params* params) {
    return new gtsam::ISAM2(*params);
}
gtsam::ISAM2Result* ISAM2_update(
    gtsam::ISAM2* p,
    const gtsam::NonlinearFactorGraph* newFactors,  //
    const gtsam::Values* newTheta) {
    return new gtsam::ISAM2Result(p->update(*newFactors, *newTheta));
}
gtsam::Values* ISAM2_calculateEstimate(const gtsam::ISAM2* p) {
    return new gtsam::Values(p->calculateEstimate());
}
}