#include <memory>
extern "C" {
void* shared_ptr_get(std::shared_ptr<void*>* p) {
    return p->get();
}
}