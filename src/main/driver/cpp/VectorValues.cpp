#define _USE_MATH_DEFINES
#include <cmath>
#include <gtsam/linear/VectorValues.h>

extern "C" {
void VectorValues_delete(gtsam::VectorValues* p) {
    delete p;
}
}