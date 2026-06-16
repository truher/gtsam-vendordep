#include <gtsam/inference/Factor.h>
#include <gtsam/nonlinear/ISAM2Result.h>

extern "C" {
void ISAM2Result_delete(gtsam::ISAM2Result* p) {
    delete p;
}
gtsam::FactorIndices* ISAM2Result_newFactorsIndices(gtsam::ISAM2Result* p) {
    return new gtsam::FactorIndices(p->newFactorsIndices);
}
void ISAM2Result_print(gtsam::ISAM2Result* p) {
    p->print();
}
}