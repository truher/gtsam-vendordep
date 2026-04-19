#include <gtsam/base/Matrix.h>

extern "C" {
gtsam::Matrix* Matrix() {
    return new gtsam::Matrix();
}
void Matrix_delete(gtsam::Matrix* m) {
    delete m;
}
gtsam::Matrix* Matrix_Matrix3(gtsam::Matrix3* m) {
    return new gtsam::Matrix(*m);
}
double Matrix_at(const gtsam::Matrix* m, int r, int c) {
    return (*m)(r, c);
}
gtsam::Vector* Matrix_diagonal_cwiseSqrt(const gtsam::Matrix* m) {
    return new gtsam::Vector(m->diagonal().cwiseSqrt());
}
int Matrix_rows(const gtsam::Matrix* m) {
    return m->rows();
}
int Matrix_cols(const gtsam::Matrix* m) {
    return m->cols();
}
}