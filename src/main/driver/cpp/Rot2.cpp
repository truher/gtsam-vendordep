#include <gtsam/geometry/Rot2.h>

extern "C" {
gtsam::Rot2* Rot2(double theta) {
    return new gtsam::Rot2(theta);
}
void Rot2_delete(gtsam::Rot2* p) {
    delete p;
}
double Rot2_theta(const gtsam::Rot2* p) {
    return p->theta();
}
double Rot2_c(const gtsam::Rot2* p) {
    return p->c();
}
double Rot2_s(const gtsam::Rot2* p) {
    return p->s();
}
}