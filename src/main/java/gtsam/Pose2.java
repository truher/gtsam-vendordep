package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose2 extends ForeignObject implements LieGroup<Pose2>, Manifold<Pose2> {
    private static final MethodHandle Pose2 = Lib.down(
            "Pose2", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Pose2_delete = Lib.downVoid(
            "Pose2_delete", ADDRESS);
    private static final MethodHandle Pose2DoublePoint2 = Lib.down(
            "Pose2DoublePoint2", ADDRESS, JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Pose2Rot2Point2 = Lib.down(
            "Pose2Rot2Point2", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2Matrix3 = Lib.down(
            "Pose2Matrix3", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_retract = Lib.down(
            "Pose2_retract", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_Retract = Lib.down(
            "Pose2_Retract", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_x = Lib.down(
            "Pose2_x", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Pose2_y = Lib.down(
            "Pose2_y", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Pose2_theta = Lib.down(
            "Pose2_theta", JAVA_DOUBLE, ADDRESS);
    private static final MethodHandle Pose2_t = Lib.down(
            "Pose2_t", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_r = Lib.down(
            "Pose2_r", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_localCoordinates = Lib.down(
            "Pose2_localCoordinates", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_between = Lib.down(
            "Pose2_between", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_inverse = Lib.down(
            "Pose2_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_AdjointMap = Lib.down(
            "Pose2_AdjointMap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_Expmap = Lib.down(
            "Pose2_Expmap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_log = Lib.down(
            "Pose2_log", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_print = Lib.downVoid(
            "Pose2_print", ADDRESS);
    private static final MethodHandle Pose2_equals = Lib.down(
            "Pose2_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Pose2_compose = Lib.down(
            "Pose2_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_matrix = Lib.down(
            "Pose2_matrix", ADDRESS, ADDRESS);

    public Pose2(MemorySegment p) {
        super(p, Pose2_delete);
    }

    public Pose2() throws Throwable {
        this(0, 0, 0);
    }

    public Pose2(double x, double y, double theta) throws Throwable {
        this((MemorySegment) Pose2.invokeExact(x, y, theta));
    }

    public Pose2(double theta, Point2 t) throws Throwable {
        this((MemorySegment) Pose2DoublePoint2.invokeExact(theta, t.ptr));
    }

    /** Copies the arguments. */
    public Pose2(Rot2 r, Point2 t) throws Throwable {
        this((MemorySegment) Pose2Rot2Point2.invokeExact(r.ptr, t.ptr));
    }

    public Pose2(Matrix3 T) throws Throwable {
        this((MemorySegment) Pose2Matrix3.invokeExact(T.ptr));
    }

    @Override
    public <T extends ForeignObject> Pose2 retract(T v) throws Throwable {
        return new Pose2((MemorySegment) Pose2_retract.invokeExact(ptr, v.ptr()));
    }

    public static Pose2 Retract(Pose2 origin, Vector3 v, Matrix Horigin, Matrix Hv) throws Throwable {
        return new Pose2((MemorySegment) Pose2_Retract.invokeExact(origin.ptr, v.ptr, Horigin.ptr, Hv.ptr));
    }

    public double x() throws Throwable {
        return (double) Pose2_x.invokeExact(ptr);
    }

    public double y() throws Throwable {
        return (double) Pose2_y.invokeExact(ptr);
    }

    public double theta() throws Throwable {
        return (double) Pose2_theta.invokeExact(ptr);
    }

    public Point2 t() throws Throwable {
        return new Point2((MemorySegment) Pose2_t.invokeExact(ptr));
    }

    public Rot2 r() throws Throwable {
        return new Rot2((MemorySegment) Pose2_r.invokeExact(ptr));
    }

    // maybe this should be Vector3
    public Vector localCoordinates(Pose2 g) throws Throwable {
        return new Vector(
                (MemorySegment) Pose2_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector local(Pose2 other) throws Throwable {
        return localCoordinates(other);
    }

    public Pose2 between(Pose2 other) throws Throwable {
        return new Pose2((MemorySegment) Pose2_between.invokeExact(ptr, other.ptr));
    }

    public Pose2 inverse() throws Throwable {
        return new Pose2((MemorySegment) Pose2_inverse.invokeExact(ptr));
    }

    public Matrix3 AdjointMap() throws Throwable {
        return new Matrix3((MemorySegment) Pose2_AdjointMap.invokeExact(ptr));
    }

    /** Picks primitives out of xi, creates new Pose2 */
    public static Pose2 Expmap(Vector3 xi) throws Throwable {
        return new Pose2((MemorySegment) Pose2_Expmap.invokeExact(xi.ptr));
    }

    public Vector3 log(Pose2 p) throws Throwable {
        return new Vector3((MemorySegment) Pose2_log.invokeExact(ptr, p.ptr));
    }

    @Override
    public String toString() {
        try {
            return String.format("%f %f %f", x(), y(), theta());
        } catch (Throwable e) {
            e.printStackTrace();
            return "";
        }
    }

    @Override
    public int dimension() {
        return 3;
    }

    public void print() throws Throwable {
        Pose2_print.invokeExact(ptr);
    }

    public boolean equals(Pose2 other, double tol) throws Throwable {
        return (boolean) Pose2_equals.invokeExact(ptr, other.ptr, tol);
    }

    public Pose2 compose(Pose2 other) throws Throwable {
        return new Pose2((MemorySegment) Pose2_compose.invokeExact(ptr, other.ptr));
    }

    public Matrix3 matrix() throws Throwable {
        return new Matrix3((MemorySegment) Pose2_matrix.invokeExact(ptr));
    }

}
