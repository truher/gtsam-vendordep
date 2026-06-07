#include <gtsam/base/Matrix.h>

#include <iostream>
#include <random>

extern "C" {
gtsam::Matrix* Matrix() {
    gtsam::Matrix* m = new gtsam::Matrix();
    m->setZero();
    return m;
}
gtsam::Matrix* Matrix_identity1() {
    return new gtsam::Matrix(gtsam::I_1x1);
}
gtsam::Matrix* Matrix_identity2() {
    return new gtsam::Matrix(gtsam::I_2x2);
}
gtsam::Matrix* Matrix_identity3() {
    return new gtsam::Matrix(gtsam::I_3x3);
}
gtsam::Matrix* Matrix_identity4() {
    return new gtsam::Matrix(gtsam::I_4x4);
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
gtsam::Vector* Matrix_timesVector(const gtsam::Matrix* m,
                                  const gtsam::Vector* v) {
    return new gtsam::Vector((*m) * (*v));
}
gtsam::Vector3* Matrix_timesVector3(const gtsam::Matrix* m,
                                    const gtsam::Vector3* v) {
    return new gtsam::Vector3(gtsam::Matrix3(*m) * (*v));
}
gtsam::Matrix* Matrix_timesDouble(const gtsam::Matrix* m, double a) {
    return new gtsam::Matrix((*m) * a);
}
bool Matrix_linear_dependent(const gtsam::Matrix* A, const gtsam::Matrix* B,
                             double tol) {
    return gtsam::linear_dependent(*A, *B, tol);
}
void Matrix_print(const gtsam::Matrix* A) {
    gtsam::print(*A);
}
// https://ricoruotongjia.medium.com/how-to-generate-gaussian-distributions-in-c-7ac880962031
std::mt19937 rng(42);
std::normal_distribution<double> dist(0.0, 1.0);
gtsam::Vector* Matrix_draw(const gtsam::Matrix* covariances) {
    Eigen::LLT<Eigen::MatrixXd> llt(*covariances);
    gtsam::Vector vec = gtsam::Vector::Zero(covariances->rows());
    Eigen::ComputationInfo info = llt.info();
    if (info != Eigen::ComputationInfo::Success) {
        std::cout << "Matrix_draw failed with info: " << info << std::endl;
        return new gtsam::Vector(vec);
    }
    gtsam::Matrix L = llt.matrixL();
    for (int i = 0; i < vec.size(); ++i) {
        vec[i] = dist(rng);
    }
    return new gtsam::Vector(L * vec);
}
}