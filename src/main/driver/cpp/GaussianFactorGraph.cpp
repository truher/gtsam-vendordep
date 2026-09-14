#include <gtsam/linear/GaussianFactorGraph.h>

extern "C" {
gtsam::VectorValues* GaussianFactorGraph_optimize(gtsam::GaussianFactorGraph* g) {
    return new gtsam::VectorValues(g->optimize());
}
}