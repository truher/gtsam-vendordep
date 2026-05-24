package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot3 extends ForeignObject {
    private static final MethodHandle Rot3Point3 = Lib.down(
            "Rot3Point3", ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3 = Lib.down(
            "Rot3", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Rot3Matrix3 = Lib.down(
            "Rot3Matrix3", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_delete = Lib.downVoid(
            "Rot3_delete", ADDRESS);
    private static final MethodHandle Rot3_Ypr = Lib.down(
            "Rot3_Ypr", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Rot3_Rodrigues = Lib.down(
            "Rot3_Rodrigues", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Rot3_AxisAngle = Lib.down(
            "Rot3_AxisAngle", ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Rot3_inverse = Lib.down(
            "Rot3_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_check_group_invariants = Lib.down(
            "Rot3_check_group_invariants", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Rot3_check_manifold_invariants = Lib.down(
            "Rot3_check_manifold_invariants", JAVA_BOOLEAN, ADDRESS, ADDRESS);

    public Rot3(MemorySegment p) {
        super(p, Rot3_delete);
    }

    public Rot3(Point3 col1, Point3 col2, Point3 col3) throws Throwable {
        this((MemorySegment) Rot3Point3.invokeExact(
                col1.ptr, col2.ptr, col3.ptr));
    }

    public Rot3() throws Throwable {
        this(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public Rot3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) Rot3.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public Rot3(Matrix3 R) throws Throwable {
        this((MemorySegment) Rot3Matrix3.invokeExact(R.ptr));
    }

    public static Rot3 Ypr(double y, double p, double r) throws Throwable {
        return new Rot3((MemorySegment) Rot3_Ypr.invokeExact(y, p, r));
    }

    public static Rot3 Rodrigues(double wx, double wy, double wz) throws Throwable {
        return new Rot3((MemorySegment) Rot3_Rodrigues.invokeExact(wx, wy, wz));
    }

    public static Rot3 AxisAngle(Point3 axis, double angle) throws Throwable {
        return new Rot3((MemorySegment) Rot3_AxisAngle.invokeExact(axis.ptr, angle));
    }

    public Rot3 inverse() throws Throwable {
        return new Rot3((MemorySegment) Rot3_inverse.invokeExact(ptr));
    }

    public static boolean check_group_invariants(Rot3 a, Rot3 b, double tol) throws Throwable {
        return (boolean) Rot3_check_group_invariants.invokeExact(a.ptr, b.ptr, tol);
    }

    public static boolean check_manifold_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) Rot3_check_manifold_invariants.invokeExact(a.ptr, b.ptr);
    }

}
