#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>

extern "C" {
//
// BASE
//
bool noiseModel_Base_isUnit(const gtsam::noiseModel::Base* model) {
    return model->isUnit();
}
double noiseModel_Base_squaredMahalanobisDistance(
    const gtsam::noiseModel::Base* model, const gtsam::Vector* v) {
    return model->squaredMahalanobisDistance(*v);
}
void noiseModel_Base_whitenInPlace(const gtsam::noiseModel::Base* model,
                                   gtsam::Vector* v) {
    model->whitenInPlace(*v);
}
void noiseModel_Base_unwhitenInPlace(const gtsam::noiseModel::Base* model,
                                     gtsam::Vector* v) {
    model->unwhitenInPlace(*v);
}
//
// GAUSSIAN
//
std::shared_ptr<gtsam::noiseModel::Gaussian>*
noiseModel_Gaussian_SqrtInformation(const gtsam::Matrix* R, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Gaussian>(
        gtsam::noiseModel::Gaussian::SqrtInformation(*R, smart));
}
std::shared_ptr<gtsam::noiseModel::Gaussian>* noiseModel_Gaussian_Covariance(
    const gtsam::Matrix* covariance, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Gaussian>(
        gtsam::noiseModel::Gaussian::Covariance(*covariance, smart));
}
std::shared_ptr<gtsam::noiseModel::Gaussian>* noiseModel_Gaussian_Information(
    const gtsam::Matrix* M, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Gaussian>(
        gtsam::noiseModel::Gaussian::Information(*M, smart));
}
gtsam::Vector* noiseModel_Gaussian_sigmas(
    const gtsam::noiseModel::Gaussian* model) {
    return new gtsam::Vector(model->sigmas());
}
gtsam::Vector* noiseModel_Gaussian_whiten(
    const gtsam::noiseModel::Gaussian* model, const gtsam::Vector* v) {
    return new gtsam::Vector(model->whiten(*v));
}
gtsam::Vector* noiseModel_Gaussian_unwhiten(
    const gtsam::noiseModel::Gaussian* model, const gtsam::Vector* v) {
    return new gtsam::Vector(model->unwhiten(*v));
}
gtsam::Matrix* noiseModel_Gaussian_R(const gtsam::noiseModel::Gaussian* model) {
    return new gtsam::Matrix(model->R());
}
gtsam::Matrix* noiseModel_Gaussian_covariance(
    const gtsam::noiseModel::Gaussian* model) {
    return new gtsam::Matrix(model->covariance());
}
gtsam::Matrix* noiseModel_Gaussian_information(
    const gtsam::noiseModel::Gaussian* model) {
    return new gtsam::Matrix(model->information());
}
gtsam::Matrix* noiseModel_Gaussian_Whiten(
    const gtsam::noiseModel::Gaussian* model, const gtsam::Matrix* H) {
    return new gtsam::Matrix(model->Whiten(*H));
}
void noiseModel_Gaussian_WhitenInPlace(const gtsam::noiseModel::Gaussian* model,
                                       gtsam::Matrix* H) {
    model->WhitenInPlace(*H);
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Gaussian_QR(
    const gtsam::noiseModel::Gaussian* model, gtsam::Matrix* Ab) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(model->QR(*Ab));
}
//
// DIAGONAL
//
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector(
    const gtsam::Vector* v, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v, smart));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector1(
    const gtsam::Vector1* v, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v, smart));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector2(
    const gtsam::Vector2* v, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v, smart));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>* noiseModel_Diagonal_SigmasVector3(
    const gtsam::Vector3* v, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Sigmas(*v, smart));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>*
noiseModel_Diagonal_VariancesVector3(const gtsam::Vector3* variances,
                                     bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Variances(*variances, smart));
}
std::shared_ptr<gtsam::noiseModel::Diagonal>*
noiseModel_Diagonal_PrecisionsVector3(const gtsam::Vector3* precisions,
                                      bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
        gtsam::noiseModel::Diagonal::Precisions(*precisions, smart));
}
//
// ISOTROPIC
//
std::shared_ptr<gtsam::noiseModel::Isotropic>* noiseModel_Isotropic_Sigma(
    int dim, double sigma, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Isotropic>(
        gtsam::noiseModel::Isotropic::Sigma(dim, sigma, smart));
}
std::shared_ptr<gtsam::noiseModel::Isotropic>* noiseModel_Isotropic_Variance(
    int dim, double variance, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Isotropic>(
        gtsam::noiseModel::Isotropic::Variance(dim, variance, smart));
}
std::shared_ptr<gtsam::noiseModel::Isotropic>* noiseModel_Isotropic_Precision(
    int dim, double precision, bool smart) {
    return new std::shared_ptr<gtsam::noiseModel::Isotropic>(
        gtsam::noiseModel::Isotropic::Precision(dim, precision, smart));
}
//
// UNIT
//

std::shared_ptr<gtsam::noiseModel::Unit>* noiseModel_Unit_Create(int dim) {
    return new std::shared_ptr<gtsam::noiseModel::Unit>(
        gtsam::noiseModel::Unit::Create(dim));
}
//
// CONSTRAINED
//
std::shared_ptr<gtsam::noiseModel::Constrained>*
noiseModel_Constrained_MixedSigmasVectorVector(gtsam::Vector* mu,
                                               gtsam::Vector* sigmas) {
    return new std::shared_ptr<gtsam::noiseModel::Constrained>(
        gtsam::noiseModel::Constrained::MixedSigmas(*mu, *sigmas));
}
std::shared_ptr<gtsam::noiseModel::Constrained>*
noiseModel_Constrained_MixedSigmasVector(gtsam::Vector* sigmas) {
    return new std::shared_ptr<gtsam::noiseModel::Constrained>(
        gtsam::noiseModel::Constrained::MixedSigmas(*sigmas));
}
//
// ROBUST
//
std::shared_ptr<gtsam::noiseModel::Robust>* noiseModel_Robust_Create(
    std::shared_ptr<gtsam::noiseModel::mEstimator::Base>* robust,
    std::shared_ptr<gtsam::noiseModel::Base>* noise) {
    return new std::shared_ptr<gtsam::noiseModel::Robust>(
        gtsam::noiseModel::Robust::Create(*robust, *noise));
}
gtsam::Vector* noiseModel_Robust_sigmas(
    const gtsam::noiseModel::Robust* model) {
    return new gtsam::Vector(model->sigmas());
}
gtsam::Vector* noiseModel_Robust_whiten(const gtsam::noiseModel::Robust* model,
                                        const gtsam::Vector* v) {
    return new gtsam::Vector(model->whiten(*v));
}
gtsam::Matrix* noiseModel_Robust_Whiten(const gtsam::noiseModel::Robust* model,
                                        const gtsam::Matrix* H) {
    return new gtsam::Matrix(model->Whiten(*H));
}
gtsam::Vector* noiseModel_Robust_unwhiten(
    const gtsam::noiseModel::Robust* model, const gtsam::Vector* v) {
    return new gtsam::Vector(model->unwhiten(*v));
}
//
// mEstimator
//
std::shared_ptr<gtsam::noiseModel::mEstimator::Huber>*
noiseModel_mEstimator_Huber_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::Huber>(
        gtsam::noiseModel::mEstimator::Huber::Create(k));
}
}