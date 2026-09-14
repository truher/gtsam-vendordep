#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/linear/NoiseModel.h>

extern "C" {
bool Testable_assert_equal_Cal3DS2(  //
    const gtsam::Cal3DS2* expected, const gtsam::Cal3DS2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Cal3DS2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Double(  //
    double expected, double actual, double tol) {
    return gtsam::assert_equal<double>(expected, actual, tol);
}
bool Testable_assert_equal_Gaussian(  //
    const gtsam::noiseModel::Gaussian* expected,
    const gtsam::noiseModel::Gaussian* actual, double tol) {
    return gtsam::assert_equal<gtsam::noiseModel::Gaussian>(  //
        *expected, *actual, tol);
}
bool Testable_assert_equal_Rot2(  //
    const gtsam::Rot2* expected, const gtsam::Rot2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Rot2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Rot3(  //
    const gtsam::Rot3* expected, const gtsam::Rot3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Rot3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Point2(  //
    const gtsam::Point2* expected, const gtsam::Point2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Point2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Pose2(  //
    const gtsam::Pose2* expected, const gtsam::Pose2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Pose2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Pose3(  //
    const gtsam::Pose3* expected, const gtsam::Pose3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Pose3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Matrix(  //
    const gtsam::Matrix* expected, const gtsam::Matrix* actual, double tol) {
    return gtsam::assert_equal<gtsam::Matrix>(*expected, *actual, tol);
}
bool Testable_assert_equal_Matrix2(  //
    const gtsam::Matrix2* expected, const gtsam::Matrix2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Matrix2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Matrix3(  //
    const gtsam::Matrix3* expected, const gtsam::Matrix3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Matrix3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Point3(  //
    const gtsam::Point3* expected, const gtsam::Point3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Point3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Unit3(  //
    const gtsam::Unit3* expected, const gtsam::Unit3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Unit3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector(  //
    const gtsam::Vector* expected, const gtsam::Vector* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector1(  //
    const gtsam::Vector1* expected, const gtsam::Vector1* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector1>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector2(  //
    const gtsam::Vector2* expected, const gtsam::Vector2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector3(  //
    const gtsam::Vector3* expected, const gtsam::Vector3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector4(  //
    const gtsam::Vector4* expected, const gtsam::Vector4* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector4>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector6(  //
    const gtsam::Vector6* expected, const gtsam::Vector6* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector6>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector9(  //
    const gtsam::Vector9* expected, const gtsam::Vector9* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector9>(*expected, *actual, tol);
}
}