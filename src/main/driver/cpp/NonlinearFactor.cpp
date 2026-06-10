#include <gtsam/nonlinear/NonlinearFactor.h>

extern "C" {
gtsam::GaussianFactor::shared_ptr* NonlinearFactor_linearize(
    const gtsam::NonlinearFactor* p,  //
    const gtsam::Values* v) {         //
    return new std::shared_ptr<gtsam::GaussianFactor>(p->linearize(*v));
}
double NonlinearFactor_error(gtsam::NonlinearFactor* p,  //
                             const gtsam::Values* v) {   //
    return p->error(*v);
}
void NonlinearFactor_print(const gtsam::NonlinearFactor* f) {
    f->print();
}
bool NonlinearFactor_equals(const gtsam::NonlinearFactor* f,  //
                            const gtsam::NonlinearFactor* g) {
    return f->equals(*g);
}
double NoiseModelFactor_weight(const gtsam::NoiseModelFactor* f,  //
                               const gtsam::Values* v) {          //
    return f->weight(*v);
}
}