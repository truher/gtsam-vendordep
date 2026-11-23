#define _USE_MATH_DEFINES
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>

#include <cmath>

extern "C" {
gtsam::LevenbergMarquardtParams* LevenbergMarquardtParams() {
    return new gtsam::LevenbergMarquardtParams();
}
void LevenbergMarquardtParams_delete(gtsam::LevenbergMarquardtParams* p) {
    delete p;
}
void LevenbergMarquardtParams_SetLegacyDefaults(
    gtsam::LevenbergMarquardtParams* p) {
    gtsam::LevenbergMarquardtParams::SetLegacyDefaults(p);
}
void LevenbergMarquardtParams_SetCeresDefaults(
    gtsam::LevenbergMarquardtParams* p) {
    gtsam::LevenbergMarquardtParams::SetCeresDefaults(p);
}
int LevenbergMarquardtParams_getVerbosityLM(
    gtsam::LevenbergMarquardtParams* p) {
    return static_cast<int>(p->verbosityLM);
}
void LevenbergMarquardtParams_setVerbosityLM(  //
    gtsam::LevenbergMarquardtParams* p, int v) {
    p->verbosityLM =
        static_cast<gtsam::LevenbergMarquardtParams::VerbosityLM>(v);
}
}