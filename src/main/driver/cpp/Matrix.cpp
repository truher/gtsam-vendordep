#include <gtsam/base/Matrix.h>
#include <iostream>

extern "C" {
gtsam::Matrix* Matrix() {
    gtsam::Matrix* m = new gtsam::Matrix();
    m->setZero();
    return m;
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
void Matrix_set(gtsam::Matrix* m, int row, int col, double v) {
    (*m)(row, col) = v;
}
void Matrix_setCol(gtsam::Matrix* m, int col, gtsam::Vector* v) {
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
gtsam::Matrix* Matrix_inverse(const gtsam::Matrix* m) {
    return new gtsam::Matrix(m->inverse());
}
gtsam::Matrix* Matrix_compose(const gtsam::Matrix* a, const gtsam::Matrix* b) {
    return new gtsam::Matrix((*a) * (*b));
}
gtsam::Matrix* Matrix_transpose(const gtsam::Matrix* m) {
    return new gtsam::Matrix(m->transpose());
}
bool Matrix_equals(const gtsam::Matrix* a, const gtsam::Matrix* b, double tol) {
    return gtsam::equal_with_abs_tol(*a, *b, tol);
}
}