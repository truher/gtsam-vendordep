#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>

extern "C" {
bool Testable_assert_equal_Rot2(  //
    const gtsam::Rot2* expected, const gtsam::Rot2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Rot2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Point2(  //
    const gtsam::Point2* expected, const gtsam::Point2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Point2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Pose2(  //
    const gtsam::Pose2* expected, const gtsam::Pose2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Pose2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Cal3DS2(  //
    const gtsam::Cal3DS2* expected, const gtsam::Cal3DS2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Cal3DS2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Pose3(  //
    const gtsam::Pose3* expected, const gtsam::Pose3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Pose3>(*expected, *actual, tol);
}
bool Testable_assert_equal_Matrix(  //
    const gtsam::Matrix* expected, const gtsam::Matrix* actual, double tol) {
    return gtsam::assert_equal<gtsam::Matrix>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector(  //
    const gtsam::Vector* expected, const gtsam::Vector* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector2(  //
    const gtsam::Vector2* expected, const gtsam::Vector2* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector2>(*expected, *actual, tol);
}
bool Testable_assert_equal_Vector3(  //
    const gtsam::Vector3* expected, const gtsam::Vector3* actual, double tol) {
    return gtsam::assert_equal<gtsam::Vector3>(*expected, *actual, tol);
}
}