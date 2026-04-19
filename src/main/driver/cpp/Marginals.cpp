#include <gtsam/nonlinear/Marginals.h>

extern "C" {
gtsam::Marginals* Marginals(const gtsam::NonlinearFactorGraph* graph,
                            const gtsam::Values* result) {
    return new gtsam::Marginals(*graph, *result);
}
void Marginals_delete(gtsam::Marginals* p) {
    delete p;
}
gtsam::Matrix* Marginals_marginalCovariance(const gtsam::Marginals* p,
                                            const gtsam::Key key) {
    return new gtsam::Matrix(p->marginalCovariance(key));
}
}