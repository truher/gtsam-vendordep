package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.noiseModel.Gaussian;

public class Testable {
    public enum FF {
        Testable_assert_equal_Cal3DS2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Double(JAVA_BOOLEAN, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Testable_assert_equal_Gaussian(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Matrix(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Matrix2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Matrix3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Point2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Point3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Pose2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Pose3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Rot2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Rot3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Unit3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector1(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector4(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector6(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector9(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public static boolean assert_equal(Cal3DS2 expected, Cal3DS2 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Cal3DS2.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Cal3DS2 expected, Cal3DS2 actual) {
        return (boolean) FF.Testable_assert_equal_Cal3DS2.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(double expected, double actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Double.invoke(expected, actual, tol);
    }

    public static boolean assert_equal(Gaussian expected, Gaussian actual, double tol)
            {
        return (boolean) FF.Testable_assert_equal_Gaussian.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Gaussian expected, Gaussian actual)
            {
        return (boolean) FF.Testable_assert_equal_Gaussian.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix expected, Matrix actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Matrix.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix expected, Matrix actual) {
        return (boolean) FF.Testable_assert_equal_Matrix.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix2 expected, Matrix2 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Matrix2.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix2 expected, Matrix2 actual) {
        return (boolean) FF.Testable_assert_equal_Matrix2.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix3 expected, Matrix3 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Matrix3.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix3 expected, Matrix3 actual) {
        return (boolean) FF.Testable_assert_equal_Matrix3.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Point2 expected, Point2 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Point2.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Point2 expected, Point2 actual) {
        return (boolean) FF.Testable_assert_equal_Point2.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Point3 expected, Point3 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Point3.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Point3 expected, Point3 actual) {
        return (boolean) FF.Testable_assert_equal_Point3.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Pose2 expected, Pose2 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Pose2.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Pose2 expected, Pose2 actual) {
        return (boolean) FF.Testable_assert_equal_Pose2.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Pose3 expected, Pose3 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Pose3.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Pose3 expected, Pose3 actual) {
        return (boolean) FF.Testable_assert_equal_Pose3.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Rot2 expected, Rot2 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Rot2.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Rot2 expected, Rot2 actual) {
        return (boolean) FF.Testable_assert_equal_Rot2.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Rot3 expected, Rot3 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Rot3.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Rot3 expected, Rot3 actual) {
        return (boolean) FF.Testable_assert_equal_Rot3.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Unit3 expected, Unit3 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Unit3.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector expected, Vector actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector expected, Vector actual) {
        return (boolean) FF.Testable_assert_equal_Vector.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector1 expected, Vector1 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector1.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector1 expected, Vector1 actual) {
        return (boolean) FF.Testable_assert_equal_Vector1.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector2 expected, Vector2 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector2.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector2 expected, Vector2 actual) {
        return (boolean) FF.Testable_assert_equal_Vector2.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector3 expected, Vector3 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector3.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector3 expected, Vector3 actual) {
        return (boolean) FF.Testable_assert_equal_Vector3.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector4 expected, Vector4 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector4.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector4 expected, Vector4 actual) {
        return (boolean) FF.Testable_assert_equal_Vector4.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector6 expected, Vector6 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector6.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector6 expected, Vector6 actual) {
        return (boolean) FF.Testable_assert_equal_Vector6.invoke(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector9 expected, Vector9 actual, double tol) {
        return (boolean) FF.Testable_assert_equal_Vector9.invoke(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector9 expected, Vector9 actual) {
        return (boolean) FF.Testable_assert_equal_Vector9.invoke(expected.ptr, actual.ptr, 1e-9);
    }

}
