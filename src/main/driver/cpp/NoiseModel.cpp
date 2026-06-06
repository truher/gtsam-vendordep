#include <gtsam/base/Matrix.h>
// supplies necessary manifold traits!
#include <gtsam/base/VectorSpace.h>
#include <gtsam/linear/NoiseModel.h>

// TODO: wrap all the invalid_argument exceptions with nullptr returns.
extern "C" {
//
// UTIL
//
// TODO: figure out why this scalar doesn't work.
bool noiseModel_matchesDimensionDouble(const gtsam::noiseModel::Base* model,
                                       const double measured) {
    return gtsam::noiseModel::matchesDimension(*model, measured);
}
bool noiseModel_matchesDimensionVector(const gtsam::noiseModel::Base* model,
                                       const gtsam::Vector* measured) {
    return gtsam::noiseModel::matchesDimension(*model, *measured);
}
bool noiseModel_matchesDimensionMatrix(const gtsam::noiseModel::Base* model,
                                       const gtsam::Matrix* measured) {
    return gtsam::noiseModel::matchesDimension(*model, *measured);
}
//
// BASE
//
bool noiseModel_Base_isUnit(const gtsam::noiseModel::Base* model) {
    return model->isUnit();
}
int noiseModel_Base_dim(const gtsam::noiseModel::Base* model) {
    return model->dim();
}
double noiseModel_Base_squaredMahalanobisDistance(
    const gtsam::noiseModel::Base* model, const gtsam::Vector* v) {
    return model->squaredMahalanobisDistance(*v);
}
double noiseModel_Base_loss(const gtsam::noiseModel::Base* model, double d) {
    return model->loss(d);
}
void noiseModel_Base_WhitenSystem(const gtsam::noiseModel::Base* model,
                                  gtsam::Matrix* A, gtsam::Vector* b) {
    return model->WhitenSystem(*A, *b);
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
double noiseModel_Gaussian_negLogConstant(
    const gtsam::noiseModel::Gaussian* model) {
    return model->negLogConstant();
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
    try {
        return new std::shared_ptr<gtsam::noiseModel::Diagonal>(
            gtsam::noiseModel::Diagonal::Sigmas(*v, smart));
    } catch (const std::invalid_argument& e) {
        std::cout << "caught err: " << e.what() << std::endl;
        return nullptr;
    }
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
    try {
        return new std::shared_ptr<gtsam::noiseModel::Isotropic>(
            gtsam::noiseModel::Isotropic::Sigma(dim, sigma, smart));
    } catch (const std::invalid_argument& e) {
        std::cout << "caught err: " << e.what() << std::endl;
        return nullptr;
    }
}
std::shared_ptr<gtsam::noiseModel::Isotropic>* noiseModel_Isotropic_Variance(
    int dim, double variance, bool smart) {
    try {
        return new std::shared_ptr<gtsam::noiseModel::Isotropic>(
            gtsam::noiseModel::Isotropic::Variance(dim, variance, smart));
    } catch (const std::invalid_argument& e) {
        std::cout << "caught err: " << e.what() << std::endl;
        return nullptr;
    }
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
std::shared_ptr<gtsam::noiseModel::Unit>* noiseModel_Unit_CreateVector(
    const gtsam::Vector* v) {
    return new std::shared_ptr<gtsam::noiseModel::Unit>(
        gtsam::noiseModel::Unit::Create(*v));
}
std::shared_ptr<gtsam::noiseModel::Unit>* noiseModel_Unit_CreateMatrix(
    const gtsam::Matrix* m) {
    return new std::shared_ptr<gtsam::noiseModel::Unit>(
        gtsam::noiseModel::Unit::Create(*m));
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
std::shared_ptr<gtsam::noiseModel::Constrained>* noiseModel_Constrained_AllInt(
    int dim) {
    return new std::shared_ptr<gtsam::noiseModel::Constrained>(
        gtsam::noiseModel::Constrained::All(dim));
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
std::shared_ptr<gtsam::noiseModel::mEstimator::Base> noiseModel_Robust_robust(
    const gtsam::noiseModel::Robust* r) {
    return new shared_ptr<gtsam::noiseModel::mEstimator::Base>(r->robust());
}
//
// mEstimator
//
double noiseModel_mEstimator_Base_weight(          //
    const gtsam::noiseModel::mEstimator::Base* b,  //
    double distance) {                             //
    return b->weight(distance);
}
std::shared_ptr<gtsam::noiseModel::mEstimator::Huber>*
noiseModel_mEstimator_Huber_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::Huber>(
        gtsam::noiseModel::mEstimator::Huber::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::Fair>*
noiseModel_mEstimator_Fair_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::Fair>(
        gtsam::noiseModel::mEstimator::Fair::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::Cauchy>*
noiseModel_mEstimator_Cauchy_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::Cauchy>(
        gtsam::noiseModel::mEstimator::Cauchy::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::AsymmetricCauchy>*
noiseModel_mEstimator_AsymmetricCauchy_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::AsymmetricCauchy>(
        gtsam::noiseModel::mEstimator::AsymmetricCauchy::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::Tukey>*
noiseModel_mEstimator_Tukey_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::Tukey>(
        gtsam::noiseModel::mEstimator::Tukey::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::AsymmetricTukey>*
noiseModel_mEstimator_AsymmetricTukey_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::AsymmetricTukey>(
        gtsam::noiseModel::mEstimator::AsymmetricTukey::Create(k));
}

std::shared_ptr<gtsam::noiseModel::mEstimator::Welsch>*
noiseModel_mEstimator_Welsch_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::Welsch>(
        gtsam::noiseModel::mEstimator::Welsch::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::GemanMcClure>*
noiseModel_mEstimator_GemanMcClure_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::GemanMcClure>(
        gtsam::noiseModel::mEstimator::GemanMcClure::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::DCS>*
noiseModel_mEstimator_DCS_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::DCS>(
        gtsam::noiseModel::mEstimator::DCS::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::L2WithDeadZone>*
noiseModel_mEstimator_L2WithDeadZone_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::L2WithDeadZone>(
        gtsam::noiseModel::mEstimator::L2WithDeadZone::Create(k));
}
std::shared_ptr<gtsam::noiseModel::mEstimator::TruncatedLeastSquares>*
noiseModel_mEstimator_TruncatedLeastSquares_Create(double k) {
    return new std::shared_ptr<gtsam::noiseModel::mEstimator::TruncatedLeastSquares>(
        gtsam::noiseModel::mEstimator::TruncatedLeastSquares::Create(k));
}
}