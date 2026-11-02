package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose2 extends ForeignObject implements LieGroup<Pose2, Vector3> {
    public enum FF {
        Pose2(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Pose2_delete(null, ADDRESS),
        Pose2DoublePoint2(ADDRESS, JAVA_DOUBLE, ADDRESS),
        Pose2Rot2Point2(ADDRESS, ADDRESS, ADDRESS),
        Pose2Matrix3(ADDRESS, ADDRESS),
        Pose2Vector3(ADDRESS, ADDRESS),
        Pose2_retract(ADDRESS, ADDRESS, ADDRESS),
        Pose2_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_Retract(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_x(JAVA_DOUBLE, ADDRESS),
        Pose2_y(JAVA_DOUBLE, ADDRESS),
        Pose2_theta(JAVA_DOUBLE, ADDRESS),
        Pose2_t(ADDRESS, ADDRESS),
        Pose2_r(ADDRESS, ADDRESS),
        Pose2_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Pose2_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_between(ADDRESS, ADDRESS, ADDRESS),
        Pose2_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_inverse(ADDRESS, ADDRESS),
        Pose2_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Pose2_AdjointMap(ADDRESS, ADDRESS),
        Pose2_logmap(ADDRESS, ADDRESS, ADDRESS),
        Pose2_expmap(ADDRESS, ADDRESS, ADDRESS),
        Pose2_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_Adjoint(ADDRESS, ADDRESS, ADDRESS),
        Pose2_compose(ADDRESS, ADDRESS, ADDRESS),
        Pose2_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_matrix(ADDRESS, ADDRESS),
        Pose2_logmap_default(ADDRESS, ADDRESS, ADDRESS),
        Pose2_expmap_default(ADDRESS, ADDRESS, ADDRESS),
        Pose2_transformTo(ADDRESS, ADDRESS, ADDRESS),
        Pose2_transformToH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_transformFrom(ADDRESS, ADDRESS, ADDRESS),
        Pose2_transformFromH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_ExpmapDerivative(ADDRESS, ADDRESS),
        Pose2_translation(ADDRESS, ADDRESS, ADDRESS),
        Pose2_bearingPoint2(ADDRESS, ADDRESS, ADDRESS),
        Pose2_bearingPoint2H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_bearingPose2(ADDRESS, ADDRESS, ADDRESS),
        Pose2_bearingPose2H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_rangePoint2(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Pose2_rangePoint2H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_rangePose2(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Pose2_rangePose2H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Pose2_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Pose2_rotation(ADDRESS, ADDRESS),
        Pose2_rotationH(ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Vector3 dxZero() {
        return new Vector3(0, 0, 0);
    }

    public Pose2(MemorySegment p) {
        super(p, FF.Pose2_delete.h);
    }

    public Pose2() {
        this(0, 0, 0);
    }

    public Pose2(double x, double y, double theta) {
        this((MemorySegment) FF.Pose2.invoke(x, y, theta));
    }

    public Pose2(double theta, Point2 t) {
        this((MemorySegment) FF.Pose2DoublePoint2.invoke(theta, t.ptr));
    }

    /** Copies the arguments. */
    public Pose2(Rot2 r, Point2 t) {
        this((MemorySegment) FF.Pose2Rot2Point2.invoke(r.ptr, t.ptr));
    }

    public Pose2(Matrix3 T) {
        this((MemorySegment) FF.Pose2Matrix3.invoke(T.ptr));
    }

    public Pose2(Vector3 v) {
        this((MemorySegment) FF.Pose2Vector3.invoke(v.ptr));
    }

    @Override
    public Pose2 retract(Vector3 v) {
        return new Pose2((MemorySegment) FF.Pose2_retract.invoke(ptr, v.ptr()));
    }

    @Override
    public Pose2 retract(Vector3 v, Matrix H1, Matrix H2) {
        return new Pose2((MemorySegment) FF.Pose2_retractH.invoke(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    public static Pose2 Retract(Pose2 origin, Vector3 v, Matrix Horigin, Matrix Hv) {
        return new Pose2((MemorySegment) FF.Pose2_Retract.invoke(origin.ptr, v.ptr, Horigin.ptr, Hv.ptr));
    }

    public double x() {
        return (double) FF.Pose2_x.invoke(ptr);
    }

    public double y() {
        return (double) FF.Pose2_y.invoke(ptr);
    }

    public double theta() {
        return (double) FF.Pose2_theta.invoke(ptr);
    }

    public Point2 t() {
        return new Point2((MemorySegment) FF.Pose2_t.invoke(ptr));
    }

    public Rot2 r() {
        return new Rot2((MemorySegment) FF.Pose2_r.invoke(ptr));
    }

    @Override
    public Vector3 local(Pose2 g) {
        return new Vector3(
                (MemorySegment) FF.Pose2_localCoordinates.invoke(ptr, g.ptr));
    }

    @Override
    public Vector3 local(Pose2 g, Matrix H1, Matrix H2) {
        return new Vector3(
                (MemorySegment) FF.Pose2_localCoordinatesH.invoke(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Pose2 between(Pose2 other) {
        return new Pose2((MemorySegment) FF.Pose2_between.invoke(ptr, other.ptr));
    }

    public Pose2 between(Pose2 other, Matrix H1, Matrix H2) {
        return new Pose2((MemorySegment) FF.Pose2_betweenH.invoke(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Pose2 inverse() {
        return new Pose2((MemorySegment) FF.Pose2_inverse.invoke(ptr));
    }

    public Pose2 inverse(Matrix H) {
        return new Pose2((MemorySegment) FF.Pose2_inverseH.invoke(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() {
        // coerce Matrix3 to dynamic. */
        return new Matrix((MemorySegment) FF.Pose2_AdjointMap.invoke(ptr));
    }

    @Override
    public Pose2 expmap(Vector3 v) {
        return new Pose2((MemorySegment) FF.Pose2_expmap.invoke(ptr, v.ptr));
    }

    @Override
    public Pose2 expmap(Vector3 v, Matrix H1, Matrix H2) {
        return new Pose2((MemorySegment) FF.Pose2_expmapH.invoke(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    public Vector3 logmap(Pose2 g) {
        return new Vector3((MemorySegment) FF.Pose2_logmap.invoke(ptr, g.ptr));
    }

    @Override
    public Vector3 logmap(Pose2 g, Matrix H1, Matrix H2) {
        return new Vector3((MemorySegment) FF.Pose2_logmapH.invoke(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Vector3 Adjoint(Vector3 v) {
        return new Vector3((MemorySegment) FF.Pose2_Adjoint.invoke(ptr, v.ptr));
    }

    @Override
    public String toString() {
        return String.format("%f %f %f", x(), y(), theta());

    }

    @Override
    public int dimension() {
        return 3;
    }

    public Pose2 compose(Pose2 other) {
        return new Pose2((MemorySegment) FF.Pose2_compose.invoke(ptr, other.ptr));
    }

    public Pose2 compose(Pose2 other, Matrix H1, Matrix H2) {
        return new Pose2((MemorySegment) FF.Pose2_composeH.invoke(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    public Matrix3 matrix() {
        return new Matrix3((MemorySegment) FF.Pose2_matrix.invoke(ptr));
    }

    public static Vector3 logmap_default(Pose2 p, Pose2 q) {
        return new Vector3((MemorySegment) FF.Pose2_logmap_default.invoke(p.ptr, q.ptr));
    }

    public static Pose2 expmap_default(Pose2 p, Vector3 d) {
        return new Pose2((MemorySegment) FF.Pose2_expmap_default.invoke(p.ptr, d.ptr));
    }

    public Point2 transformTo(Point2 point) {
        return new Point2((MemorySegment) FF.Pose2_transformTo.invoke(
                ptr, point.ptr));
    }

    public Point2 transformTo(//
            Point2 point, //
            Matrix Dpose, //
            Matrix Dpoint) {
        return new Point2((MemorySegment) FF.Pose2_transformToH.invoke(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public Point2 transformFrom(Point2 point) {
        return new Point2((MemorySegment) FF.Pose2_transformFrom.invoke(
                ptr, point.ptr));
    }

    public Point2 transformFrom(//
            Point2 point, //
            Matrix Dpose, //
            Matrix Dpoint) {
        return new Point2((MemorySegment) FF.Pose2_transformFromH.invoke(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public static Matrix3 ExpmapDerivative(Vector3 v) {
        return new Matrix3((MemorySegment) FF.Pose2_ExpmapDerivative.invoke(v.ptr));
    }

    public Point2 translation(Matrix H) {
        return new Point2((MemorySegment) FF.Pose2_translation.invoke(ptr, H.ptr));
    }

    public Rot2 bearing(Point2 p) {
        return new Rot2((MemorySegment) FF.Pose2_bearingPoint2.invoke(ptr, p.ptr));
    }

    public Rot2 bearing(Point2 p, Matrix H1, Matrix H2) {
        return new Rot2((MemorySegment) FF.Pose2_bearingPoint2H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Rot2 bearing(Pose2 p) {
        return new Rot2((MemorySegment) FF.Pose2_bearingPose2.invoke(ptr, p.ptr));
    }

    public Rot2 bearing(Pose2 p, Matrix H1, Matrix H2) {
        return new Rot2((MemorySegment) FF.Pose2_bearingPose2H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public double range(Point2 p) {
        return (double) FF.Pose2_rangePoint2.invoke(ptr, p.ptr);
    }

    public double range(Point2 p, Matrix H1, Matrix H2) {
        return (double) FF.Pose2_rangePoint2H.invoke(ptr, p.ptr, H1.ptr, H2.ptr);
    }

    public double range(Pose2 p) {
        return (double) FF.Pose2_rangePose2.invoke(ptr, p.ptr);
    }

    public double range(Pose2 p, Matrix H1, Matrix H2) {
        return (double) FF.Pose2_rangePose2H.invoke(ptr, p.ptr, H1.ptr, H2.ptr);
    }

    public static boolean check_group_invariants(Pose2 a, Pose2 b) {
        return (boolean) FF.Pose2_check_group_invariants.invoke(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Pose2 a, Pose2 b) {
        return (boolean) FF.Pose2_check_manifold_invariants.invoke(a.ptr, b.ptr);
    }

    public Rot2 rotation() {
        return new Rot2((MemorySegment) FF.Pose2_rotation.invoke(ptr));
    }

    public Rot2 rotation(Matrix H) {
        return new Rot2((MemorySegment) FF.Pose2_rotationH.invoke(ptr, H.ptr));
    }

}
