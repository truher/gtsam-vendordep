#include <gtsam/base/types.h>
#include <gtsam/inference/Symbol.h>

/**
 * See Symbol.h.
 * TODO: remove the duplication here.
 */
extern "C" {
void symbol_print(gtsam::Key key) {
    gtsam::Symbol(key).print();
}
gtsam::Key symbol_shorthand_A(std::uint64_t j) {
    return gtsam::symbol_shorthand::A(j);
}
gtsam::Key symbol_shorthand_B(std::uint64_t j) {
    return gtsam::symbol_shorthand::B(j);
}
gtsam::Key symbol_shorthand_C(std::uint64_t j) {
    return gtsam::symbol_shorthand::C(j);
}
gtsam::Key symbol_shorthand_D(std::uint64_t j) {
    return gtsam::symbol_shorthand::D(j);
}
gtsam::Key symbol_shorthand_E(std::uint64_t j) {
    return gtsam::symbol_shorthand::E(j);
}
gtsam::Key symbol_shorthand_F(std::uint64_t j) {
    return gtsam::symbol_shorthand::F(j);
}
gtsam::Key symbol_shorthand_G(std::uint64_t j) {
    return gtsam::symbol_shorthand::G(j);
}
gtsam::Key symbol_shorthand_H(std::uint64_t j) {
    return gtsam::symbol_shorthand::H(j);
}
gtsam::Key symbol_shorthand_I(std::uint64_t j) {
    return gtsam::symbol_shorthand::I(j);
}
gtsam::Key symbol_shorthand_J(std::uint64_t j) {
    return gtsam::symbol_shorthand::J(j);
}
gtsam::Key symbol_shorthand_K(std::uint64_t j) {
    return gtsam::symbol_shorthand::K(j);
}
gtsam::Key symbol_shorthand_L(std::uint64_t j) {
    return gtsam::symbol_shorthand::L(j);
}
gtsam::Key symbol_shorthand_M(std::uint64_t j) {
    return gtsam::symbol_shorthand::M(j);
}
gtsam::Key symbol_shorthand_N(std::uint64_t j) {
    return gtsam::symbol_shorthand::N(j);
}
gtsam::Key symbol_shorthand_O(std::uint64_t j) {
    return gtsam::symbol_shorthand::O(j);
}
gtsam::Key symbol_shorthand_P(std::uint64_t j) {
    return gtsam::symbol_shorthand::P(j);
}
gtsam::Key symbol_shorthand_Q(std::uint64_t j) {
    return gtsam::symbol_shorthand::Q(j);
}
gtsam::Key symbol_shorthand_R(std::uint64_t j) {
    return gtsam::symbol_shorthand::R(j);
}
gtsam::Key symbol_shorthand_S(std::uint64_t j) {
    return gtsam::symbol_shorthand::S(j);
}
gtsam::Key symbol_shorthand_T(std::uint64_t j) {
    return gtsam::symbol_shorthand::T(j);
}
gtsam::Key symbol_shorthand_U(std::uint64_t j) {
    return gtsam::symbol_shorthand::U(j);
}
gtsam::Key symbol_shorthand_V(std::uint64_t j) {
    return gtsam::symbol_shorthand::V(j);
}
gtsam::Key symbol_shorthand_W(std::uint64_t j) {
    return gtsam::symbol_shorthand::W(j);
}
gtsam::Key symbol_shorthand_X(std::uint64_t j) {
    return gtsam::symbol_shorthand::X(j);
}
gtsam::Key symbol_shorthand_Y(std::uint64_t j) {
    return gtsam::symbol_shorthand::Y(j);
}
gtsam::Key symbol_shorthand_Z(std::uint64_t j) {
    return gtsam::symbol_shorthand::Z(j);
}
}