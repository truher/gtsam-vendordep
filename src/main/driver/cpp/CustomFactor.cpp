#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/CustomFactor.h>

extern "C" {

/**
 * The error function signature is
 * std::function<Vector(
 *   const CustomFactor &,
 *   const Values &,
 *   const JacobianVector *)>
 *
 * My error function uses pointers, so we take the
 * address of the reference arguments.  These pointers should not be
 * owned by the error function.
 *
 * Using "address of local" for the error function arguments is
 * ok because the referents will stick around during the error
 * function execution.
 *
 * This copies the return value, which is a heap-allocated,
 * java-owned thing, because it will be deleted soon after the
 * error function completes.
 * 
 * Making this copy is quite slow (about 10% in my test), so it
 * would be good to find a way to eliminate it.
 */
std::shared_ptr<gtsam::CustomFactor>* CustomFactor(
    const gtsam::SharedNoiseModel* noiseModel,                        //
    const gtsam::KeyVector* keys,                                     //
    gtsam::Vector* (*errorFunction)(const gtsam::CustomFactor*,       //
                                    const gtsam::Values*,             //
                                    const gtsam::JacobianVector*)) {  //
    return new std::shared_ptr<gtsam::CustomFactor>(                  //
        new gtsam::CustomFactor(                                      //
            *noiseModel,                                              //
            *keys,                                                    //
            [errorFunction](const gtsam::CustomFactor& factor,        //
                            const gtsam::Values& v,                   //
                            const gtsam::JacobianVector* H) -> gtsam::Vector {
                return gtsam::Vector(*(errorFunction(&factor, &v, H)));
            }));
}
const gtsam::KeyVector* CustomFactor_keys(const gtsam::CustomFactor* p) {
    return new gtsam::KeyVector(p->keys());
}
double CustomFactor_error(gtsam::CustomFactor* p, const gtsam::Values* v) {
    return p->error(*v);
}
}