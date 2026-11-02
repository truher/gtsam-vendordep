package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Pose3 extends ForeignObject implements LieGroup<Pose3, Vector6> {
    public enum FF {
        Pose3(ADDRESS, ADDRESS, ADDRESS),
        Pose3_delete(null, ADDRESS),
        Pose3_Pose2(ADDRESS, ADDRESS),
        Pose3_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Pose3_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_compose(ADDRESS, ADDRESS, ADDRESS),
        Pose3_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_retract(ADDRESS, ADDRESS, ADDRESS),
        Pose3_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_between(ADDRESS, ADDRESS, ADDRESS),
        Pose3_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_inverse(ADDRESS, ADDRESS),
        Pose3_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Pose3_AdjointMap(ADDRESS, ADDRESS),
        Pose3_logmap(ADDRESS, ADDRESS, ADDRESS),
        Pose3_expmap(ADDRESS, ADDRESS, ADDRESS),
        Pose3_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_matrix(ADDRESS, ADDRESS),
        Pose3_bearingPoint3(ADDRESS, ADDRESS, ADDRESS),
        Pose3_bearingPoint3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_bearingPose3(ADDRESS, ADDRESS, ADDRESS),
        Pose3_bearingPose3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_rangePoint3(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Pose3_rangePoint3H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_rangePose3(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Pose3_rangePose3H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Pose3_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Pose3_logmap_default(ADDRESS, ADDRESS, ADDRESS),
        Pose3_expmap_default(ADDRESS, ADDRESS, ADDRESS),
        Pose3_Adjoint(ADDRESS, ADDRESS, ADDRESS),
        Pose3_AdjointH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_rotation(ADDRESS, ADDRESS),
        Pose3_rotationH(ADDRESS, ADDRESS, ADDRESS),
        Pose3_translation(ADDRESS, ADDRESS),
        Pose3_translationH(ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformTo(ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformToH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformFrom(ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformFromH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_interpolate(ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Pose3_interpolateH(ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS),
        Pose3_interpolateRt(ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Pose3_interpolateRtH(ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformPoseFrom(ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformPoseFromH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformPoseTo(ADDRESS, ADDRESS, ADDRESS),
        Pose3_transformPoseToH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose3_Create(ADDRESS, ADDRESS, ADDRESS),
        Pose3_CreateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Vector6 dxZero() {
        return new Vector6(0, 0, 0, 0, 0, 0);
    }

    public Pose3(MemorySegment p) {
        super(p, FF.Pose3_delete.h);
    }

    public Pose3() {
        // TODO: avoid these
        this(new Rot3(), new Point3(0, 0, 0));
    }

    /** Copies the arguments. */
    public Pose3(Rot3 r, Point3 t) {
        this((MemorySegment) FF.Pose3.invoke(r.ptr, t.ptr));
    }

    public Pose3(Pose2 p) {
        this((MemorySegment) FF.Pose3_Pose2.invoke(p.ptr));
    }

    @Override
    public Vector6 local(Pose3 g) {
        return new Vector6(
                (MemorySegment) FF.Pose3_localCoordinates.invoke(ptr, g.ptr));
    }

    @Override
    public Vector6 local(Pose3 g, Matrix H1, Matrix H2) {
        return new Vector6(
                (MemorySegment) FF.Pose3_localCoordinates.invoke(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Pose3 compose(Pose3 p2) {
        return new Pose3((MemorySegment) FF.Pose3_compose.invoke(ptr, p2.ptr));
    }

    public Pose3 compose(Pose3 p2, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_composeH.invoke(
                ptr, p2.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public int dimension() {
        return 6;
    }

    @Override
    public Pose3 retract(Vector6 v) {
        return new Pose3((MemorySegment) FF.Pose3_retract.invoke(ptr, v.ptr()));
    }

    @Override
    public Pose3 retract(Vector6 v, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_retractH.invoke(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Pose3 inverse() {
        return new Pose3((MemorySegment) FF.Pose3_inverse.invoke(ptr));
    }

    public Pose3 inverse(Matrix H) {
        return new Pose3((MemorySegment) FF.Pose3_inverseH.invoke(ptr, H.ptr));
    }

    @Override
    public Pose3 between(Pose3 other) {
        return new Pose3((MemorySegment) FF.Pose3_between.invoke(ptr, other.ptr));
    }

    @Override
    public Pose3 between(Pose3 other, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_betweenH.invoke(
                ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Matrix AdjointMap() {
        // coerce Matrix6 to dynamic
        return new Matrix((MemorySegment) FF.Pose3_AdjointMap.invoke(ptr));
    }

    @Override
    public Pose3 expmap(Vector6 v) {
        return new Pose3((MemorySegment) FF.Pose3_expmap.invoke(ptr, v.ptr));
    }

    @Override
    public Pose3 expmap(Vector6 v, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_expmapH.invoke(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    public Vector6 logmap(Pose3 g) {
        return new Vector6((MemorySegment) FF.Pose3_logmap.invoke(ptr, g.ptr));
    }

    @Override
    public Vector6 logmap(Pose3 g, Matrix H1, Matrix H2) {
        return new Vector6((MemorySegment) FF.Pose3_logmapH.invoke(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    /** 4x4 homogeneous matrix */
    public Matrix matrix() {
        return new Matrix((MemorySegment) FF.Pose3_matrix.invoke(ptr));
    }

    public Unit3 bearing(Point3 p) {
        return new Unit3((MemorySegment) FF.Pose3_bearingPoint3.invoke(ptr, p.ptr));
    }

    public Unit3 bearing(Point3 p, Matrix H1, Matrix H2) {
        return new Unit3((MemorySegment) FF.Pose3_bearingPoint3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Unit3 bearing(Pose3 p) {
        return new Unit3((MemorySegment) FF.Pose3_bearingPose3.invoke(ptr, p.ptr));
    }

    public Unit3 bearing(Pose3 p, Matrix H1, Matrix H2) {
        return new Unit3((MemorySegment) FF.Pose3_bearingPose3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public double range(Point3 p) {
        return (double) FF.Pose3_rangePoint3.invoke(ptr, p.ptr);
    }

    public double range(Point3 p, Matrix H1, Matrix H2) {
        return (double) FF.Pose3_rangePoint3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr);
    }

    public double range(Pose3 p) {
        return (double) FF.Pose3_rangePose3.invoke(ptr, p.ptr);
    }

    public double range(Pose3 p, Matrix H1, Matrix H2) {
        return (double) FF.Pose3_rangePose3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr);
    }

    public static boolean check_group_invariants(Pose3 a, Pose3 b) {
        return (boolean) FF.Pose3_check_group_invariants.invoke(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Pose3 a, Pose3 b) {
        return (boolean) FF.Pose3_check_manifold_invariants.invoke(a.ptr, b.ptr);
    }

    public static Vector6 logmap_default(Pose3 p, Pose3 q) {
        return new Vector6((MemorySegment) FF.Pose3_logmap_default.invoke(p.ptr, q.ptr));
    }

    public static Pose3 expmap_default(Pose3 p, Vector6 d) {
        return new Pose3((MemorySegment) FF.Pose3_expmap_default.invoke(p.ptr, d.ptr));
    }

    public Vector6 Adjoint(Vector6 v) {
        return new Vector6((MemorySegment) FF.Pose3_Adjoint.invoke(ptr, v.ptr));
    }

    public Vector6 Adjoint(Vector6 v, Matrix H1, Matrix H2) {
        return new Vector6((MemorySegment) FF.Pose3_AdjointH.invoke(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    public Rot3 rotation() {
        return new Rot3((MemorySegment) FF.Pose3_rotation.invoke(ptr));
    }

    public Rot3 rotation(Matrix H) {
        return new Rot3((MemorySegment) FF.Pose3_rotationH.invoke(ptr, H.ptr));
    }

    public Point3 translation() {
        return new Point3((MemorySegment) FF.Pose3_translation.invoke(ptr));
    }

    public Point3 translation(Matrix H) {
        return new Point3((MemorySegment) FF.Pose3_translationH.invoke(ptr, H.ptr));
    }

    public Point3 transformTo(Point3 point) {
        return new Point3((MemorySegment) FF.Pose3_transformTo.invoke(
                ptr, point.ptr));
    }

    public Point3 transformTo(//
            Point3 point, //
            Matrix Dpose, //
            Matrix Dpoint) {
        return new Point3((MemorySegment) FF.Pose3_transformToH.invoke(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public Point3 transformFrom(Point3 point) {
        return new Point3((MemorySegment) FF.Pose3_transformFrom.invoke(
                ptr, point.ptr));
    }

    public Point3 transformFrom(//
            Point3 point, //
            Matrix Dpose, //
            Matrix Dpoint) {
        return new Point3((MemorySegment) FF.Pose3_transformFromH.invoke(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public static Pose3 interpolate(Pose3 X, Pose3 Y, double t) {
        return new Pose3((MemorySegment) FF.Pose3_interpolate.invoke(X.ptr, Y.ptr, t));
    }

    public static Pose3 interpolate(
            Pose3 X, Pose3 Y, double t, Matrix H1, Matrix H2, Matrix H3) {
        return new Pose3((MemorySegment) FF.Pose3_interpolateH.invoke(X.ptr, Y.ptr, t, H1.ptr, H2.ptr, H3.ptr));
    }

    public Pose3 interpolateRt(Pose3 Y, double t) {
        return new Pose3((MemorySegment) FF.Pose3_interpolateRt.invoke(ptr, Y.ptr, t));
    }

    public Pose3 interpolateRt(Pose3 Y, double t, Matrix H1, Matrix H2, Matrix H3) {
        return new Pose3((MemorySegment) FF.Pose3_interpolateRtH.invoke(ptr, Y.ptr, t, H1.ptr, H2.ptr, H3.ptr));
    }

    public Pose3 transformPoseFrom(Pose3 p) {
        return new Pose3((MemorySegment) FF.Pose3_transformPoseFrom.invoke(ptr, p.ptr));
    }

    public Pose3 transformPoseFrom(Pose3 p, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_transformPoseFromH.invoke(
                ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Pose3 transformPoseTo(Pose3 p) {
        return new Pose3((MemorySegment) FF.Pose3_transformPoseTo.invoke(ptr, p.ptr));
    }

    public Pose3 transformPoseTo(Pose3 p, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_transformPoseToH.invoke(
                ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public static Pose3 Create(Rot3 R, Point3 t) {
        return new Pose3((MemorySegment) FF.Pose3_Create.invoke(R.ptr, t.ptr));
    }

    public static Pose3 Create(Rot3 R, Point3 t, Matrix H1, Matrix H2) {
        return new Pose3((MemorySegment) FF.Pose3_CreateH.invoke(R.ptr, t.ptr, H1.ptr, H2.ptr));
    }

}
