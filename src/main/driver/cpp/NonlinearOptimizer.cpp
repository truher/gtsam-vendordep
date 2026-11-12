#define _USE_MATH_DEFINES
#include <cmath>
#include <gtsam/nonlinear/NonlinearOptimizer.h>

extern "C" {
gtsam::Values* NonlinearOptimizer_optimize(gtsam::NonlinearOptimizer* o) {
    return new gtsam::Values(o->optimize());
}
}