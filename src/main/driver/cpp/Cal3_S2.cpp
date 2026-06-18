#include <gtsam/geometry/Cal3_S2.h>

extern "C" {
    void Cal3_S2_delete(gtsam::Cal3_S2* p) {
    delete p;
}
}