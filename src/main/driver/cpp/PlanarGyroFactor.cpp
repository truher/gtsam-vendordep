#include <gtsam/navigation/PlanarGyroFactor.h>

extern "C" {
std::shared_ptr<gtsam::PlanarGyroParams>* PlanarGyroParams(
    double arw, double biasInstability) {
    return new std::shared_ptr<gtsam::PlanarGyroParams>(
        new gtsam::PlanarGyroParams(arw, biasInstability));
}
void PlanarGyroParams_delete(std::shared_ptr<gtsam::PlanarGyroParams>* obj) {
    // decrements the shared_ptr counter
    delete obj;
}
double PlanarGyroParams_arwSigma(gtsam::PlanarGyroParams* p, double dt) {
    // TODO: make arwSigma const in gtsam
    return p->arwSigma(dt);
}
//
//
//
std::shared_ptr<gtsam::PlanarGyroBiasFactor>* PlanarGyroParams(
    const gtsam::Key bias_i,  //
    const gtsam::Key bias_j,  //
    const std::shared_ptr<gtsam::PlanarGyroParams>* p) {
    return new std::shared_ptr<gtsam::PlanarGyroBiasFactor>(
        new gtsam::PlanarGyroBiasFactor(bias_i, bias_j, *p));
}
void PlanarGyroBiasFactor_delete(
    std::shared_ptr<gtsam::PlanarGyroBiasFactor>* obj) {
    // decrements the shared_ptr counter
    delete obj;
}
//
//
//

void PlanarGyroFactor_delete(std::shared_ptr<gtsam::PlanarGyroFactor>* obj) {
    // decrements the shared_ptr counter
    delete obj;
}
std::shared_ptr<gtsam::PlanarGyroFactor>* PlanarGyroFactor_FromRotation(  //
    const gtsam::Key pose_i,                                              //
    const gtsam::Key pose_j,                                              //
    const gtsam::Key bias,                                                //
    const std::shared_ptr<gtsam::PlanarGyroParams>* p,                    //
    const gtsam::Rot2* dr,                                                //
    double dt) {                                                          //
    return new std::shared_ptr<gtsam::PlanarGyroFactor>(
        new gtsam::PlanarGyroFactor(gtsam::PlanarGyroFactor::FromRotation(  //
            pose_i, pose_j, bias, *p, *dr, dt)));
}
std::shared_ptr<gtsam::PlanarGyroFactor>* PlanarGyroFactor_FromRate(  //
    const gtsam::Key pose_i,                                          //
    const gtsam::Key pose_j,                                          //
    const gtsam::Key bias,                                            //
    const std::shared_ptr<gtsam::PlanarGyroParams>* p,                //
    double omega,                                                     //
    double dt) {                                                      //
    return new std::shared_ptr<gtsam::PlanarGyroFactor>(
        new gtsam::PlanarGyroFactor(gtsam::PlanarGyroFactor::FromRate(  //
            pose_i, pose_j, bias, *p, omega, dt)));
}
gtsam::Rot2* PlanarGyroFactor_deltaR(const gtsam::PlanarGyroFactor* p,
                                     double bias, gtsam::Matrix* H) {
    return new gtsam::Rot2(p->deltaR(bias, *H));
}
gtsam::Rot2* PlanarGyroFactor_predict(const gtsam::PlanarGyroFactor* p,  //
                                      const gtsam::Rot2* Ri,             //
                                      double bias,                       //
                                      gtsam::Matrix* H1,                 //
                                      gtsam::Matrix* H2) {               //
    return new gtsam::Rot2(p->predict(*Ri, bias, *H1, *H2));
}
double PlanarGyroFactor_computeError(const gtsam::PlanarGyroFactor* p,  //
                                     const gtsam::Rot2* Ri,             //
                                     const gtsam::Rot2* Rj,             //
                                     double bias,                       //
                                     gtsam::Matrix* H1,                 //
                                     gtsam::Matrix* H2,                 //
                                     gtsam::Matrix* H3) {               //
    return p->computeError(*Ri, *Rj, bias, *H1, *H2, *H3);
}
gtsam::Vector* PlanarGyroFactor_evaluateError(  //
    const gtsam::PlanarGyroFactor* f,           //
    const gtsam::Pose2* Pi,                     //
    const gtsam::Pose2* Pj,                     //
    const double bias,                          //
    gtsam::Matrix* H1,                          //
    gtsam::Matrix* H2,                          //
    gtsam::Matrix* H3) {                        //
    return new gtsam::Vector(f->evaluateError(*Pi, *Pj, bias, *H1, *H2, *H3));
}
}