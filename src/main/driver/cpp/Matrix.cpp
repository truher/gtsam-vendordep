#include <gtsam/base/Matrix.h>

extern "C" {
gtsam::Matrix* Matrix() {
    return new gtsam::Matrix();
}
void Matrix_delete(gtsam::Matrix* m) {
    delete m;
}
gtsam::Matrix* Matrix_withRowsCols(int rows, int cols) {
    return new Eigen::MatrixXd(rows, cols);
}
gtsam::Matrix* Matrix_Matrix3(gtsam::Matrix3* m) {
    return new gtsam::Matrix(*m);
}
void Matrix_setCol(gtsam::Matrix* m, double col, gtsam::Vector* v) {
    (*m).col(col) = (*v);
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