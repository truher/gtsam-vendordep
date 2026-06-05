#include <memory>
/**
 *  shared_ptr does not care about the parameter type.
 */
extern "C" {
void* shared_ptr_get(std::shared_ptr<void>* p) {
    return p->get();
}
void shared_ptr_delete(std::shared_ptr<void>* obj) {
    delete obj;
}
}