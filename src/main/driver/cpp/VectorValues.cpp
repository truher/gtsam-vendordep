#include <gtsam/linear/VectorValues.h>

extern "C" {
void VectorValues_delete(gtsam::VectorValues* p) {
    delete p;
}
}