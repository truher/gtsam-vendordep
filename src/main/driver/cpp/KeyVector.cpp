#include <gtsam/inference/Key.h>

extern "C" {

gtsam::KeyVector* KeyVector() {
    return new gtsam::KeyVector();
}
void KeyVector_delete(gtsam::KeyVector* p) {
    delete p;
}
void KeyVector_push_back(gtsam::KeyVector* v, gtsam::Key k) {
    v->push_back(k);
}
gtsam::Key KeyVector_at(gtsam::KeyVector* v, uint64_t i) {
    return v->at(i);
}
uint64_t KeyVector_size(gtsam::KeyVector* v) {
    return v->size();
}
}