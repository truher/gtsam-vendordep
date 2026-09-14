#include <gtsam/nonlinear/FixedLagSmoother.h>

extern "C" {
void Result_delete(gtsam::FixedLagSmoother::Result* p) {
    delete p;
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
}