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
 * TODO: add more kinds of noise
 */
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas(const gtsam::Vector* v) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas1(const gtsam::Vector1* v) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas2(const gtsam::Vector2* v) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Sigmas3(const gtsam::Vector3* v) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Diagonal::Sigmas(*v));
}
gtsam::SharedNoiseModel* SharedNoiseModel_Unit(int dim) {
    return new gtsam::SharedNoiseModel(gtsam::noiseModel::Unit::Create(dim));
}
void SharedNoiseModel_delete(gtsam::SharedNoiseModel* m) {
    delete m;
}
long SharedNoiseModel_use_count(const gtsam::SharedNoiseModel* p) {
    return p->use_count();
}
}