#include <gtsam/nonlinear/ISAM2Result.h>

extern "C" {
void ISAM2Result_delete(gtsam::ISAM2Result* p) {
    delete p;
}
gtsam::FactorIndices* ISAM2Result_newFactorsIndices(
    const gtsam::ISAM2Result* p) {
    return new gtsam::FactorIndices(p->newFactorsIndices());
}
}