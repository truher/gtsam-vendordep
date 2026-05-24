package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

public class Testable {
    private static final MethodHandle Testable_assert_equal_Rot2 = Lib.down(
            "Testable_assert_equal_Rot2", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Rot3 = Lib.down(
            "Testable_assert_equal_Rot3", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Point2 = Lib.down(
            "Testable_assert_equal_Point2", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Pose2 = Lib.down(
            "Testable_assert_equal_Pose2", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Cal3DS2 = Lib.down(
            "Testable_assert_equal_Cal3DS2", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Pose3 = Lib.down(
            "Testable_assert_equal_Pose3", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Matrix = Lib.down(
            "Testable_assert_equal_Matrix", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Matrix2 = Lib.down(
            "Testable_assert_equal_Matrix2", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Matrix3 = Lib.down(
            "Testable_assert_equal_Matrix3", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Vector = Lib.down(
            "Testable_assert_equal_Vector", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Vector1 = Lib.down(
            "Testable_assert_equal_Vector1", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Vector2 = Lib.down(
            "Testable_assert_equal_Vector2", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Testable_assert_equal_Vector3 = Lib.down(
            "Testable_assert_equal_Vector3", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);

    public static boolean assert_equal(Rot2 expected, Rot2 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Rot2.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Rot2 expected, Rot2 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Rot2.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Rot3 expected, Rot3 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Rot3.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Rot3 expected, Rot3 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Rot3.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Point2 expected, Point2 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Point2.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Point2 expected, Point2 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Point2.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Pose2 expected, Pose2 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Pose2.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Pose2 expected, Pose2 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Pose2.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Cal3DS2 expected, Cal3DS2 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Cal3DS2.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Cal3DS2 expected, Cal3DS2 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Cal3DS2.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Pose3 expected, Pose3 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Pose3.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Pose3 expected, Pose3 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Pose3.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix expected, Matrix actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Matrix.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix expected, Matrix actual) throws Throwable {
        return (boolean) Testable_assert_equal_Matrix.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix2 expected, Matrix2 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Matrix2.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix2 expected, Matrix2 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Matrix2.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Matrix3 expected, Matrix3 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Matrix3.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Matrix3 expected, Matrix3 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Matrix3.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector expected, Vector actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Vector.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector expected, Vector actual) throws Throwable {
        return (boolean) Testable_assert_equal_Vector.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector1 expected, Vector1 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Vector1.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector1 expected, Vector1 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Vector1.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector2 expected, Vector2 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Vector2.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector2 expected, Vector2 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Vector2.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

    public static boolean assert_equal(Vector3 expected, Vector3 actual, double tol) throws Throwable {
        return (boolean) Testable_assert_equal_Vector3.invokeExact(expected.ptr, actual.ptr, tol);
    }

    public static boolean assert_equal(Vector3 expected, Vector3 actual) throws Throwable {
        return (boolean) Testable_assert_equal_Vector3.invokeExact(expected.ptr, actual.ptr, 1e-9);
    }

}
