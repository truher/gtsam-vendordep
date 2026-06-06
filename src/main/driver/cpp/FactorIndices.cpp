#include <gtsam/inference/Factor.h>

// this is just a vector; maybe use vector instead?
extern "C" {
gtsam::FactorIndices* FactorIndices() {
    return new gtsam::FactorIndices();
}
void FactorIndices_delete(gtsam::FactorIndices* p) {
    delete p;
}
void FactorIndices_add(gtsam::FactorIndices* p,  //
                       gtsam::Key i) {
    p->push_back(i);
}
int FactorIndices_size(gtsam::FactorIndices* p) {
    return p->size();
}
int FactorIndices_at(gtsam::FactorIndices* p, int i) {
    return p->at(i);
}
}