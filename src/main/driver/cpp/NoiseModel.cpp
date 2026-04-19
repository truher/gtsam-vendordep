#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

extern "C" {
/**
 * Returns a pointer to a heap-allocated copy of the shared pointer made by the
 * Sigmas static method.
 *
 * This copy should be copied to java-land.
 *
 * It would be cleaner to copy the shared pointer itself into java-land, using
 * "move" semantics, but I couldn't find a way to do that.
 *
 * TODO: add deleter
 */
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas1(gtsam::Vector1* v) {
    // so, a scalar
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas2(gtsam::Vector2* v) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas3(gtsam::Vector3* v) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Unit(int dim) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Unit::Create(dim));
}
long SharedNoiseModel_use_count(gtsam::SharedNoiseModel* p) {
    return p->use_count();
}
}