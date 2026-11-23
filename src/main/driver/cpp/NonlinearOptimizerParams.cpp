#define _USE_MATH_DEFINES
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>

#include <cmath>

extern "C" {
long NonlinearOptimizerParams_getMaxIterations(
    gtsam::NonlinearOptimizerParams* p) {
    return p->maxIterations;
}
double NonlinearOptimizerParams_getRelativeErrorTol(
    gtsam::NonlinearOptimizerParams* p) {
    return p->relativeErrorTol;
}
double NonlinearOptimizerParams_getAbsoluteErrorTol(
    gtsam::NonlinearOptimizerParams* p) {
    return p->absoluteErrorTol;
}
double NonlinearOptimizerParams_getErrorTol(
    gtsam::NonlinearOptimizerParams* p) {
    return p->errorTol;
}
int NonlinearOptimizerParams_getVerbosity(  //
    gtsam::NonlinearOptimizerParams* p) {
    return static_cast<int>(p->verbosity);
}
void NonlinearOptimizerParams_setMaxIterations(
    gtsam::NonlinearOptimizerParams* p, int value) {
    p->maxIterations = value;
}
void NonlinearOptimizerParams_setRelativeErrorTol(
    gtsam::NonlinearOptimizerParams* p, double value) {
    p->relativeErrorTol = value;
}
void NonlinearOptimizerParams_setAbsoluteErrorTol(
    gtsam::NonlinearOptimizerParams* p, double value) {
    p->absoluteErrorTol = value;
}
void NonlinearOptimizerParams_setErrorTol(  //
    gtsam::NonlinearOptimizerParams* p, double value) {
    p->errorTol = value;
}
void NonlinearOptimizerParams_setVerbosity(  //
    gtsam::NonlinearOptimizerParams* p, int value) {
    p->verbosity =
        static_cast<gtsam::NonlinearOptimizerParams::Verbosity>(value);
}
}