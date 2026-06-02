#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

extern "C" {

//
// DIAGONAL
//
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector(
    const gtsam::Vector* v) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector1(
    const gtsam::Vector1* v) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector2(
    const gtsam::Vector2* v) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector3(
    const gtsam::Vector3* v) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v));
}

void noiseModel_Diagonal_delete(gtsam::SharedNoiseModel* m) {
    delete m;
}


//
// UNIT
//

std::shared_ptr<gtsam::noiseModel::Unit>* noiseModel_Unit_Create(int dim) {
    return new std::shared_ptr<gtsam::noiseModel::Unit>(
        gtsam::noiseModel::Unit::Create(dim));
}
void noiseModel_Unit_delete(std::shared_ptr<gtsam::noiseModel::Unit>* m) {
    delete m;
}
}