#include <gtsam/inference/FactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

extern "C" {
gtsam::FactorIndices* FactorGraph_add_factorsNonlinearFactorGraph(  //
    gtsam::FactorGraph<gtsam::NonlinearFactor>* p,                         //
    gtsam::NonlinearFactorGraph* g) {
    return new gtsam::FactorIndices(p->add_factors(*g));
}
}