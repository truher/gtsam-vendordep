#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

extern "C" {
gtsam::LevenbergMarquardtParams* LevenbergMarquardtParams() {
    return new gtsam::LevenbergMarquardtParams();
}
void LevenbergMarquardtParams_delete(gtsam::LevenbergMarquardtParams* p) {
    delete p;
}
}