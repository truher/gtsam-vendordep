package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose2 extends ForeignObject implements LieGroup<Pose2, Vector3> {
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
    private static final MethodHandle Pose2Vector3 = Lib.down(
            "Pose2Vector3", ADDRESS, ADDRESS);
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
    private static final MethodHandle Pose2_betweenH = Lib.down(
            "Pose2_betweenH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_inverse = Lib.down(
            "Pose2_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_inverseH = Lib.down(
            "Pose2_inverseH", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_AdjointMap = Lib.down(
            "Pose2_AdjointMap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_Adjoint = Lib.down(
            "Pose2_Adjoint", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_Expmap = Lib.down(
            "Pose2_Expmap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_ExpmapH = Lib.down(
            "Pose2_ExpmapH", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_Logmap = Lib.down(
            "Pose2_Logmap", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_LogmapH = Lib.down(
            "Pose2_LogmapH", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_logmap = Lib.down(
            "Pose2_logmap", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_compose = Lib.down(
            "Pose2_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_composeH = Lib.down(
            "Pose2_composeH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_matrix = Lib.down(
            "Pose2_matrix", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_logmap_default = Lib.down(
            "Pose2_logmap_default", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_expmap_default = Lib.down(
            "Pose2_expmap_default", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_transformTo = Lib.down(
            "Pose2_transformTo", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_transformToH = Lib.down(
            "Pose2_transformToH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_transformFrom = Lib.down(
            "Pose2_transformFrom", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_transformFromH = Lib.down(
            "Pose2_transformFromH", ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_ExpmapDerivative = Lib.down(
            "Pose2_ExpmapDerivative", ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_translation = Lib.down(
            "Pose2_translation", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_bearingPoint2 = Lib.down(
            "Pose2_bearingPoint2", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_bearingPose2 = Lib.down(
            "Pose2_bearingPose2", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_rangePoint2 = Lib.down(
            "Pose2_rangePoint2", JAVA_DOUBLE, ADDRESS, ADDRESS);
    private static final MethodHandle Pose2_rangePose2 = Lib.down(
            "Pose2_rangePose2", JAVA_DOUBLE, ADDRESS, ADDRESS);

    public static class Pose2Traits implements LieGroup.Traits<Pose2, Vector3> {

        @Override
        public Pose2 Identity() throws Throwable {
            return new Pose2();
        }

        @Override
        public Pose2 Expmap(Vector3 xi) throws Throwable {
            return new Pose2((MemorySegment) Pose2_Expmap.invokeExact(xi.ptr));
        }

        @Override
        public Vector3 Logmap(Pose2 g) throws Throwable {
            return new Vector3((MemorySegment) Pose2_Logmap.invokeExact(g.ptr));
        }
    }

    public static final Pose2Traits traits = new Pose2Traits();

    @Override
    public Pose2Traits traits() {
        return traits;
    }

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3(0, 0, 0);
    }

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

    public Pose2(Vector3 v) throws Throwable {
        this((MemorySegment) Pose2Vector3.invokeExact(v.ptr));
    }

    @Override
    public Pose2 retract(Vector3 v) throws Throwable {
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

    @Override
    public Vector3 localCoordinates(Pose2 g) throws Throwable {
        return new Vector3(
                (MemorySegment) Pose2_localCoordinates.invokeExact(ptr, g.ptr));
    }

    @Override
    public Pose2 between(Pose2 other) throws Throwable {
        return new Pose2((MemorySegment) Pose2_between.invokeExact(ptr, other.ptr));
    }

    public Pose2 between(Pose2 other, Matrix H1, Matrix H2) throws Throwable {
        return new Pose2((MemorySegment) Pose2_betweenH.invokeExact(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Pose2 inverse() throws Throwable {
        return new Pose2((MemorySegment) Pose2_inverse.invokeExact(ptr));
    }

    public Pose2 inverse(Matrix H) throws Throwable {
        return new Pose2((MemorySegment) Pose2_inverseH.invokeExact(ptr, H.ptr));
    }

    /** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) Pose2_AdjointMap.invokeExact(ptr));
    }

    public Vector3 Adjoint(Vector3 v) throws Throwable {
        return new Vector3((MemorySegment) Pose2_Adjoint.invokeExact(ptr, v.ptr));
    }

    public static Pose2 Expmap(Vector3 xi, Matrix Hv) throws Throwable {
        return new Pose2((MemorySegment) Pose2_ExpmapH.invokeExact(xi.ptr, Hv.ptr));
    }

    public static Vector3 Logmap(Pose2 p, Matrix H) throws Throwable {
        return new Vector3((MemorySegment) Pose2_LogmapH.invokeExact(p.ptr, H.ptr));
    }

    public Vector3 logmap(Pose2 p) throws Throwable {
        return new Vector3((MemorySegment) Pose2_logmap.invokeExact(ptr, p.ptr));
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

    public Pose2 compose(Pose2 other) throws Throwable {
        return new Pose2((MemorySegment) Pose2_compose.invokeExact(ptr, other.ptr));
    }

    public Pose2 compose(Pose2 other, Matrix H1, Matrix H2) throws Throwable {
        return new Pose2((MemorySegment) Pose2_composeH.invokeExact(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    public Matrix3 matrix() throws Throwable {
        return new Matrix3((MemorySegment) Pose2_matrix.invokeExact(ptr));
    }

    // TODO: maybe make this static?
    public Vector3 logmap_default(Pose2 p) throws Throwable {
        return new Vector3((MemorySegment) Pose2_logmap_default.invokeExact(ptr, p.ptr));
    }

    // TODO: maybe make this static?
    public Pose2 expmap_default(Vector3 d) throws Throwable {
        return new Pose2((MemorySegment) Pose2_expmap_default.invokeExact(ptr, d.ptr));
    }

    public Point2 transformTo(Point2 point) throws Throwable {
        return new Point2((MemorySegment) Pose2_transformTo.invokeExact(
                ptr, point.ptr));
    }

    public Point2 transformTo(//
            Point2 point, //
            Matrix Dpose, //
            Matrix Dpoint) throws Throwable {
        return new Point2((MemorySegment) Pose2_transformToH.invokeExact(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public Point2 transformFrom(Point2 point) throws Throwable {
        return new Point2((MemorySegment) Pose2_transformFrom.invokeExact(
                ptr, point.ptr));
    }

    public Point2 transformFrom(//
            Point2 point, //
            Matrix Dpose, //
            Matrix Dpoint) throws Throwable {
        return new Point2((MemorySegment) Pose2_transformFromH.invokeExact(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public static Matrix3 ExpmapDerivative(Vector3 v) throws Throwable {
        return new Matrix3((MemorySegment) Pose2_ExpmapDerivative.invokeExact(v.ptr));
    }

    public Point2 translation(Matrix H) throws Throwable {
        return new Point2((MemorySegment) Pose2_translation.invokeExact(ptr, H.ptr));
    }

    public Rot2 bearing(Point2 p) throws Throwable {
        return new Rot2((MemorySegment) Pose2_bearingPoint2.invokeExact(ptr, p.ptr));
    }

    public Rot2 bearing(Pose2 p) throws Throwable {
        return new Rot2((MemorySegment) Pose2_bearingPose2.invokeExact(ptr, p.ptr));
    }

    public double range(Point2 p) throws Throwable {
        return (double) Pose2_rangePoint2.invokeExact(ptr, p.ptr);
    }

    public double range(Pose2 p) throws Throwable {
        return (double) Pose2_rangePose2.invokeExact(ptr, p.ptr);
    }

}
