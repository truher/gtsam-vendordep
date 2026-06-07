#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

extern "C" {
gtsam::GaussNewtonParams* GaussNewtonParams() {
    return new gtsam::GaussNewtonParams();
}
void GaussNewtonParams_delete(gtsam::GaussNewtonParams* p) {
    delete p;
}
void GaussNewtonParams_relativeErrorTol(gtsam::GaussNewtonParams* p,  //
                                        double tol) {
    (*p).relativeErrorTol = tol;
}
void GaussNewtonParams_maxIterations(gtsam::GaussNewtonParams* p,  //
                                     int i) {
    (*p).maxIterations = i;
}
//
//
//
void GaussNewtonOptimizer_delete(gtsam::GaussNewtonOptimizer* p) {
    delete p;
}
gtsam::GaussNewtonOptimizer* GaussNewtonOptimizer(  //
    gtsam::NonlinearFactorGraph graph,              //
    gtsam::Values initialValues,                    //
    gtsam::GaussNewtonParams params) {              //
    return new gtsam::GaussNewtonOptimizer(*graph, *initialValues, *params);
}
}