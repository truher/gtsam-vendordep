#define _USE_MATH_DEFINES
#include <cmath>
#include <gtsam/nonlinear/FixedLagSmoother.h>

extern "C" {
void Result_delete(gtsam::FixedLagSmoother::Result* p) {
    delete p;
}
long Result_getIterations(gtsam::FixedLagSmoother::Result* p) {
    return p->getIterations();
}
double Result_getError(gtsam::FixedLagSmoother::Result* p) {
    return p->getError();
}
//
gtsam::FixedLagSmoother::KeyTimestampMap* KeyTimestampMap() {
    return new gtsam::FixedLagSmoother::KeyTimestampMap();
}
void KeyTimestampMap_delete(gtsam::FixedLagSmoother::KeyTimestampMap* p) {
    delete p;
}
// TODO: maybe move this?
void KeyTimestampMap_put(gtsam::FixedLagSmoother::KeyTimestampMap* p,
                         gtsam::Key k, double v) {
    (*p)[k] = v;
}
void KeyTimestampMap_clear(gtsam::FixedLagSmoother::KeyTimestampMap* p) {
    p->clear();
}
void KeyTimestampMap_print(const gtsam::FixedLagSmoother::KeyTimestampMap* p) {
    for (const gtsam::FixedLagSmoother::KeyTimestampMap::value_type&
             key_timestamp : *p) {
        std::cout << "Key: " << key_timestamp.first
             << "  Time: " << key_timestamp.second << std::endl;
    }
}
}