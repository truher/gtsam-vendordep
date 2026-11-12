#define _USE_MATH_DEFINES
#include <cmath>
#include <gtsam/nonlinear/Marginals.h>

extern "C" {
gtsam::Marginals* MarginalsCholesky(const gtsam::NonlinearFactorGraph* graph,
                                    const gtsam::Values* result) {
    try {
        return new gtsam::Marginals(*graph, *result,
                                    gtsam::Marginals::CHOLESKY);
    } catch (const gtsam::IndeterminateSystemException e) {
        std::cout << "caught IndeterminateSystemException: " << e.what()
                  << std::endl;
        return nullptr;
    }
}
gtsam::Marginals* MarginalsQR(const gtsam::NonlinearFactorGraph* graph,
                              const gtsam::Values* result) {
    try {
        return new gtsam::Marginals(*graph, *result, gtsam::Marginals::QR);
    } catch (const gtsam::IndeterminateSystemException e) {
        std::cout << "caught IndeterminateSystemException: " << e.what()
                  << std::endl;
        return nullptr;
    }
}
void Marginals_delete(gtsam::Marginals* p) {
    delete p;
}
gtsam::Matrix* Marginals_marginalCovariance(const gtsam::Marginals* p,
                                            const gtsam::Key key) {
    try {
        return new gtsam::Matrix(p->marginalCovariance(key));
    } catch (const std::out_of_range e) {
        std::cout << "caught std::out_of_range: " << e.what() << std::endl;
        return nullptr;
    }
}
void Marginals_print(const gtsam::Marginals* p) {
    p->print();
}
}