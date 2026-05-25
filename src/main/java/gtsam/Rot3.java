package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot3 extends ForeignObject implements LieGroup<Rot3, Vector3> {
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
    private static final MethodHandle Rot3_matrix = Lib.down(
            "Rot3_matrix", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_compose = Lib.down(
            "Rot3_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_between = Lib.down(
            "Rot3_between", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_inverse = Lib.down(
            "Rot3_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_inverseH = Lib.down(
            "Rot3_inverseH", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_transpose = Lib.down(
            "Rot3_transpose", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_localCoordinates = Lib.down(
            "Rot3_localCoordinates", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_retract = Lib.down(
            "Rot3_retract", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_logmap = Lib.down(
            "Rot3_logmap", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_expmap = Lib.down(
            "Rot3_expmap", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_Expmap = Lib.down(
            "Rot3_Expmap", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_Logmap = Lib.down(
            "Rot3_Logmap", ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_check_group_invariants = Lib.down(
            "Rot3_check_group_invariants", JAVA_BOOLEAN, ADDRESS, ADDRESS);
    private static final MethodHandle Rot3_check_manifold_invariants = Lib.down(
            "Rot3_check_manifold_invariants", JAVA_BOOLEAN, ADDRESS, ADDRESS);

    public static class Rot3Traits implements LieGroup.Traits<Rot3, Vector3> {

        @Override
        public Rot3 Identity() throws Throwable {
            return new Rot3();
        }

        @Override
        public Rot3 Expmap(Vector3 v) throws Throwable {
            return new Rot3((MemorySegment) Rot3_Expmap.invokeExact(v.ptr));
        }

        @Override
        public Vector3 Logmap(Rot3 g) throws Throwable {
            return new Vector3((MemorySegment) Rot3_Logmap.invokeExact(g.ptr));
        }
    }

    public static final Rot3Traits traits = new Rot3Traits();

    @Override
    public Traits<Rot3, Vector3> traits() {
        return traits;
    }

    public Rot3(MemorySegment p) {
        super(p, Rot3_delete);
    }

    public Rot3(Point3 col1, Point3 col2, Point3 col3) throws Throwable {
        this((MemorySegment) Rot3Point3.invokeExact(
                col1.ptr, col2.ptr, col3.ptr));
    }

    public Rot3() throws Throwable {
        this(Matrix3.identity());
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

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3();
    }

    @Override
    public int dimension() throws Throwable {
        return 3;
    }

    @Override
    public Vector3 localCoordinates(Rot3 g) throws Throwable {
        return new Vector3((MemorySegment) Rot3_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v) throws Throwable {
        return new Rot3((MemorySegment) Rot3_retract.invokeExact(ptr, v.ptr));
    }

    public Vector3 logmap(Rot3 g) throws Throwable {
        return new Vector3((MemorySegment) Rot3_logmap.invokeExact(ptr, g.ptr));
    }

    public Rot3 expmap(Vector3 v) throws Throwable {
        return new Rot3((MemorySegment) Rot3_expmap.invokeExact(ptr, v.ptr));
    }

    public Matrix3 matrix() throws Throwable {
        return new Matrix3((MemorySegment) Rot3_matrix.invokeExact(ptr));
    }

    @Override
    public Rot3 compose(Rot3 other) throws Throwable {
        return new Rot3((MemorySegment) Rot3_compose.invokeExact(ptr, other.ptr));
    }

    @Override
    public Rot3 between(Rot3 g) throws Throwable {
        return new Rot3((MemorySegment) Rot3_between.invokeExact(ptr, g.ptr));
    }

    @Override
    public Rot3 inverse() throws Throwable {
        return new Rot3((MemorySegment) Rot3_inverse.invokeExact(ptr));
    }

    public Rot3 inverse(Matrix H) throws Throwable {
        return new Rot3((MemorySegment) Rot3_inverseH.invokeExact(ptr, H.ptr));
    }

    public Matrix3 transpose() throws Throwable {
        return new Matrix3((MemorySegment) Rot3_transpose.invokeExact(ptr));
    }

    public static boolean check_group_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) Rot3_check_group_invariants.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) Rot3_check_manifold_invariants.invokeExact(a.ptr, b.ptr);
    }

}
