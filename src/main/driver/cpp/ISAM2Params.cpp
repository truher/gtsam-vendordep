#include <gtsam/nonlinear/ISAM2Params.h>

extern "C" {
gtsam::ISAM2Params* ISAM2Params() {
    return new gtsam::ISAM2Params();
}
void ISAM2Params_delete(gtsam::ISAM2Params* p) {
    delete p;
}
void ISAM2Params_findUnusedFactorSlots(gtsam::ISAM2Params* p, bool val) {
    p->findUnusedFactorSlots = val;
}
}