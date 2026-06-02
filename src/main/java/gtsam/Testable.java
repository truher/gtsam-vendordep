package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class Testable {
    public enum FF {
        Testable_assert_equal_Double(JAVA_BOOLEAN, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Testable_assert_equal_Rot2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Rot3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Point2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Pose2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Cal3DS2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Pose3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Matrix(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Matrix2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Matrix3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Point3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Unit3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector1(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector2(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector3(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector6(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Testable_assert_equal_Vector9(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static boolean assert_equal(Cal3DS2 expected, Cal3DS2 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Cal3DS2.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Cal3DS2 expected, Cal3DS2 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Cal3DS2.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(double expected, double actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Double.h.invokeExact(expected, actual, tol);
    }

    public static boolean assert_equal(Matrix expected, Matrix actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Matrix.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix expected, Matrix actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Matrix.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix2 expected, Matrix2 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Matrix2.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix2 expected, Matrix2 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Matrix2.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix3 expected, Matrix3 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Matrix3.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix3 expected, Matrix3 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Matrix3.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Point2 expected, Point2 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Point2.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Point2 expected, Point2 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Point2.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Point3 expected, Point3 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Point3.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Point3 expected, Point3 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Point3.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Pose2 expected, Pose2 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Pose2.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Pose2 expected, Pose2 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Pose2.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Pose3 expected, Pose3 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Pose3.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Pose3 expected, Pose3 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Pose3.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Rot2 expected, Rot2 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Rot2.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Rot2 expected, Rot2 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Rot2.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Rot3 expected, Rot3 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Rot3.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Rot3 expected, Rot3 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Rot3.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Unit3 expected, Unit3 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Unit3.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector expected, Vector actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector expected, Vector actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector1 expected, Vector1 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector1.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector1 expected, Vector1 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector1.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector2 expected, Vector2 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector2.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector2 expected, Vector2 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector2.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector3 expected, Vector3 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector3.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector3 expected, Vector3 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector3.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector6 expected, Vector6 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector6.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector6 expected, Vector6 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector6.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector9 expected, Vector9 actual, double tol) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector9.h.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector9 expected, Vector9 actual) throws Throwable {
        return (boolean) FF.Testable_assert_equal_Vector9.h.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

}
