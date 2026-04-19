#include <gtsam/nonlinear/NonlinearFactor.h>

extern "C" {
/**
 * Returns a pointer to a heap-allocated shared pointer to a GaussianFactor, so
 * that Java can manage the lifespan of the shared pointer.
 *
 * TODO: actually do that (e.g. with Arenas that make sense).
 */
gtsam::GaussianFactor::shared_ptr* NonlinearFactor_linearize(
    const gtsam::NonlinearFactor* p, const gtsam::Values* v) {
    gtsam::GaussianFactor::shared_ptr gf = p->linearize(*v);
    return new gtsam::GaussianFactor::shared_ptr(gf);
}
}