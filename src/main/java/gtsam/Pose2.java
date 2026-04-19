package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose2 extends ForeignObject {
    private static final MethodHandle Pose2 = Lib.down(
            "Pose2", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Pose2_delete = Lib.downVoid(
            "Pose2_delete", ADDRESS);
    private static final MethodHandle Pose2Rot2Point2 = Lib.down(
            "Pose2Rot2Point2", ADDRESS, ADDRESS, ADDRESS);
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

    public Pose2(MemorySegment p) {
        super(p, Pose2_delete);
    }

    public Pose2() throws Throwable {
        this(0, 0, 0);
    }

    public Pose2(double x, double y, double theta) throws Throwable {
        this((MemorySegment) Pose2.invokeExact(x, y, theta));
    }

    /** Copies the arguments. */
    public Pose2(Rot2 r, Point2 t) throws Throwable {
        this((MemorySegment) Pose2Rot2Point2.invokeExact(r.ptr, t.ptr));
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

    public TangentVector localCoordinates(Pose2 g) throws Throwable {
        return new TangentVector(
                (MemorySegment) Pose2_localCoordinates.invokeExact(ptr, g.ptr));
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

}
