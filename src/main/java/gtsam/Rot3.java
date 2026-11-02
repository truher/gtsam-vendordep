package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemoryLayout;
import java.lang.foreign.MemorySegment;
import java.lang.foreign.SegmentAllocator;
import java.lang.foreign.StructLayout;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;
import org.team100.foreign.Pairs;

public class Rot3 extends ForeignObject implements LieGroup<Rot3, Vector3> {
    private static final StructLayout AxisAngle = MemoryLayout.structLayout(
            ValueLayout.ADDRESS.withName("first"),
            ValueLayout.JAVA_DOUBLE.withName("second"));
    private static final VarHandle AxisAngle_first = AxisAngle
            .varHandle(MemoryLayout.PathElement.groupElement("first"));
    private static final VarHandle AxisAngle_second = AxisAngle
            .varHandle(MemoryLayout.PathElement.groupElement("second"));

    public enum FF {
        Rot3Point3(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3(ADDRESS,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3Matrix3(ADDRESS, ADDRESS),
        Rot3Quaternion(ADDRESS, ADDRESS),
        Rot3_Quaternion(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3_delete((ValueLayout) null, ADDRESS),
        Rot3_Yaw(ADDRESS, JAVA_DOUBLE),
        Rot3_Pitch(ADDRESS, JAVA_DOUBLE),
        Rot3_Roll(ADDRESS, JAVA_DOUBLE),
        Rot3_Ypr(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3_YprH(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS),
        Rot3_Rodrigues(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3_RodriguesVector3(ADDRESS, ADDRESS),
        Rot3_AxisAnglePoint3(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Rot3_AxisAngleUnit3(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Rot3_matrix(ADDRESS, ADDRESS),
        Rot3_compose(ADDRESS, ADDRESS, ADDRESS),
        Rot3_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_between(ADDRESS, ADDRESS, ADDRESS),
        Rot3_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_inverse(ADDRESS, ADDRESS),
        Rot3_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_AdjointMap(ADDRESS, ADDRESS),
        Rot3_transpose(ADDRESS, ADDRESS),
        Rot3_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Rot3_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_retract(ADDRESS, ADDRESS, ADDRESS),
        Rot3_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_logmap(ADDRESS, ADDRESS, ADDRESS),
        Rot3_expmap(ADDRESS, ADDRESS, ADDRESS),
        Rot3_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_rotateUnit3(ADDRESS, ADDRESS, ADDRESS),
        Rot3_rotatePoint3(ADDRESS, ADDRESS, ADDRESS),
        Rot3_rotateUnit3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_rotatePoint3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_unrotateUnit3(ADDRESS, ADDRESS, ADDRESS),
        Rot3_unrotatePoint3(ADDRESS, ADDRESS, ADDRESS),
        Rot3_unrotateUnit3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_unrotatePoint3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Rot3_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Rot3_axisAngle(AxisAngle, ADDRESS),
        Rot3_ClosestTo(ADDRESS, ADDRESS),
        Rot3_Rx(ADDRESS, JAVA_DOUBLE),
        Rot3_Ry(ADDRESS, JAVA_DOUBLE),
        Rot3_Rz(ADDRESS, JAVA_DOUBLE),
        Rot3_RzRyRx(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot3_RzRyRxH(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS),
        Rot3_RzRyRxVector3(ADDRESS, ADDRESS),
        Rot3_RzRyRxVector3H(ADDRESS, ADDRESS, ADDRESS),
        Rot3_normalized(ADDRESS, ADDRESS),
        Rot3_roll(JAVA_DOUBLE, ADDRESS),
        Rot3_pitch(JAVA_DOUBLE, ADDRESS),
        Rot3_yaw(JAVA_DOUBLE, ADDRESS),
        Rot3_rollH(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Rot3_pitchH(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Rot3_yawH(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Rot3_xyz(ADDRESS, ADDRESS),
        Rot3_ypr(ADDRESS, ADDRESS),
        Rot3_rpy(ADDRESS, ADDRESS),
        Rot3_xyzH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_yprH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_rpyH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_RQ(Pairs.PtrPair, ADDRESS),
        Rot3_RQH(Pairs.PtrPair, ADDRESS, ADDRESS),
        Rot3_toQuaternion(ADDRESS, ADDRESS),
        Rot3_slerp(ADDRESS, ADDRESS, JAVA_DOUBLE, ADDRESS),
        Rot3_interpolate(ADDRESS, ADDRESS, ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private FF(StructLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Rot3(MemorySegment p) {
        super(p, FF.Rot3_delete.h);
    }

    public Rot3(Point3 col1, Point3 col2, Point3 col3) {
        this((MemorySegment) FF.Rot3Point3.invoke(
                col1.ptr, col2.ptr, col3.ptr));
    }

    /** Identity. */
    public Rot3() {
        this(Matrix3.identity());
    }

    public Rot3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) {
        this((MemorySegment) FF.Rot3.invoke(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public Rot3(Matrix3 R) {
        this((MemorySegment) FF.Rot3Matrix3.invoke(R.ptr));
    }

    public Rot3(Quaternion q) {
        this((MemorySegment) FF.Rot3Quaternion.invoke(q.ptr));
    }

    public static Rot3 Quaternion(double w, double x, double y, double z) {
        return new Rot3((MemorySegment) FF.Rot3_Quaternion.invoke(w, x, y, z));
    }

    public static Rot3 Yaw(double t) {
        return new Rot3((MemorySegment) FF.Rot3_Yaw.invoke(t));
    }

    public static Rot3 Pitch(double t) {
        return new Rot3((MemorySegment) FF.Rot3_Pitch.invoke(t));
    }

    public static Rot3 Roll(double t) {
        return new Rot3((MemorySegment) FF.Rot3_Roll.invoke(t));
    }

    public static Rot3 Ypr(double y, double p, double r) {
        return new Rot3((MemorySegment) FF.Rot3_Ypr.invoke(y, p, r));
    }

    public static Rot3 Ypr(double y, double p, double r, //
            Matrix H1, Matrix H2, Matrix H3) {
        return new Rot3((MemorySegment) FF.Rot3_YprH.invoke(y, p, r, H1.ptr, H2.ptr, H3.ptr));
    }

    public static Rot3 Rodrigues(double wx, double wy, double wz) {
        return new Rot3((MemorySegment) FF.Rot3_Rodrigues.invoke(wx, wy, wz));
    }

    public static Rot3 Rodrigues(Point3 v) {
        return new Rot3((MemorySegment) FF.Rot3_RodriguesVector3.invoke(v.ptr));
    }

    public static Rot3 AxisAngle(Point3 axis, double angle) {
        return new Rot3((MemorySegment) FF.Rot3_AxisAnglePoint3.invoke(axis.ptr, angle));
    }

    public static Rot3 AxisAngle(Unit3 axis, double angle) {
        return new Rot3((MemorySegment) FF.Rot3_AxisAngleUnit3.invoke(axis.ptr, angle));
    }

    @Override
    public Vector3 dxZero() {
        return new Vector3();
    }

    @Override
    public int dimension() {
        return 3;
    }

    @Override
    public Vector3 local(Rot3 g) {
        return new Vector3((MemorySegment) FF.Rot3_localCoordinates.invoke(ptr, g.ptr));
    }

    @Override
    public Vector3 local(Rot3 g, Matrix H1, Matrix H2) {
        return new Vector3((MemorySegment) FF.Rot3_localCoordinatesH.invoke(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v) {
        return new Rot3((MemorySegment) FF.Rot3_retract.invoke(ptr, v.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v, Matrix H1, Matrix H2) {
        return new Rot3((MemorySegment) FF.Rot3_retract.invoke(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    public Vector3 logmap(Rot3 g) {
        return new Vector3((MemorySegment) FF.Rot3_logmap.invoke(ptr, g.ptr));
    }

    public Rot3 expmap(Vector3 v) {
        return new Rot3((MemorySegment) FF.Rot3_expmap.invoke(ptr, v.ptr));
    }

    @Override
    public Rot3 expmap(Vector3 v, Matrix H1, Matrix H2) {
        return new Rot3((MemorySegment) FF.Rot3_expmapH.invoke(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector3 logmap(Rot3 g, Matrix H1, Matrix H2) {
        return new Vector3((MemorySegment) FF.Rot3_logmapH.invoke(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Matrix3 matrix() {
        return new Matrix3((MemorySegment) FF.Rot3_matrix.invoke(ptr));
    }

    @Override
    public Rot3 compose(Rot3 other) {
        return new Rot3((MemorySegment) FF.Rot3_compose.invoke(ptr, other.ptr));
    }

    @Override
    public Rot3 compose(Rot3 other, Matrix H1, Matrix H2) {
        return new Rot3((MemorySegment) FF.Rot3_composeH.invoke(
                ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 between(Rot3 g) {
        return new Rot3((MemorySegment) FF.Rot3_between.invoke(ptr, g.ptr));
    }

    @Override
    public Rot3 between(Rot3 g, Matrix H1, Matrix H2) {
        return new Rot3((MemorySegment) FF.Rot3_betweenH.invoke(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 inverse() {
        return new Rot3((MemorySegment) FF.Rot3_inverse.invoke(ptr));
    }

    @Override
    public Rot3 inverse(Matrix H) {
        return new Rot3((MemorySegment) FF.Rot3_inverseH.invoke(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() {
        return new Matrix((MemorySegment) FF.Rot3_AdjointMap.invoke(ptr));
    }

    public Matrix3 transpose() {
        return new Matrix3((MemorySegment) FF.Rot3_transpose.invoke(ptr));
    }

    public Unit3 rotate(Unit3 p) {
        return new Unit3((MemorySegment) FF.Rot3_rotateUnit3.invoke(ptr, p.ptr));
    }

    public Point3 rotate(Point3 p) {
        return new Point3((MemorySegment) FF.Rot3_rotatePoint3.invoke(ptr, p.ptr));
    }

    public Unit3 rotate(Unit3 p, Matrix H1, Matrix H2) {
        return new Unit3((MemorySegment) FF.Rot3_rotateUnit3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point3 rotate(Point3 p, Matrix H1, Matrix H2) {
        return new Point3((MemorySegment) FF.Rot3_rotatePoint3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Unit3 unrotate(Unit3 p) {
        return new Unit3((MemorySegment) FF.Rot3_unrotateUnit3.invoke(ptr, p.ptr));
    }

    public Point3 unrotate(Point3 p) {
        return new Point3((MemorySegment) FF.Rot3_unrotatePoint3.invoke(ptr, p.ptr));
    }

    public Unit3 unrotate(Unit3 p, Matrix H1, Matrix H2) {
        return new Unit3((MemorySegment) FF.Rot3_unrotateUnit3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point3 unrotate(Point3 p, Matrix H1, Matrix H2) {
        return new Point3((MemorySegment) FF.Rot3_unrotatePoint3H.invoke(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public static boolean check_group_invariants(Rot3 a, Rot3 b) {
        return (boolean) FF.Rot3_check_group_invariants.invoke(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot3 a, Rot3 b) {
        return (boolean) FF.Rot3_check_manifold_invariants.invoke(a.ptr, b.ptr);
    }

    public Pair<Unit3, Double> axisAngle() {
        MemorySegment resultStruct = (MemorySegment) FF.Rot3_axisAngle.invoke(
                (SegmentAllocator) Lib.arena, ptr);
        MemorySegment first = (MemorySegment) AxisAngle_first.get(resultStruct, 0);
        double second = (double) AxisAngle_second.get(resultStruct, 0);
        return new Pair<>(new Unit3(first), second);
    }

    public static Rot3 ClosestTo(Matrix3 M) {
        return new Rot3((MemorySegment) FF.Rot3_ClosestTo.invoke(M.ptr));
    }

    public static Rot3 Rx(double t) {
        return new Rot3((MemorySegment) FF.Rot3_Rx.invoke(t));
    }

    public static Rot3 Ry(double t) {
        return new Rot3((MemorySegment) FF.Rot3_Ry.invoke(t));
    }

    public static Rot3 Rz(double t) {
        return new Rot3((MemorySegment) FF.Rot3_Rz.invoke(t));
    }

    public static Rot3 RzRyRx(double x, double y, double z) {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRx.invoke(x, y, z));
    }

    public static Rot3 RzRyRx(double x, double y, double z, //
            Matrix H1, Matrix H2, Matrix H3) {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRxH.invoke(x, y, z, H1.ptr, H2.ptr, H3.ptr));
    }

    public static Rot3 RzRyRx(Vector3 xyz) {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRxVector3.invoke(xyz.ptr));
    }

    public static Rot3 RzRyRx(Vector3 xyz, Matrix H) {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRxVector3H.invoke(xyz.ptr, H.ptr));
    }

    public Rot3 normalized() {
        return new Rot3((MemorySegment) FF.Rot3_normalized.invoke(ptr));
    }

    public double roll() {
        return (double) FF.Rot3_roll.invoke(ptr);
    }

    public double pitch() {
        return (double) FF.Rot3_pitch.invoke(ptr);
    }

    public double yaw() {
        return (double) FF.Rot3_yaw.invoke(ptr);
    }

    public double roll(Matrix H) {
        return (double) FF.Rot3_rollH.invoke(ptr, H.ptr);
    }

    public double pitch(Matrix H) {
        return (double) FF.Rot3_pitchH.invoke(ptr, H.ptr);
    }

    public double yaw(Matrix H) {
        return (double) FF.Rot3_yawH.invoke(ptr, H.ptr);
    }

    public Vector3 xyz() {
        return new Vector3((MemorySegment) FF.Rot3_xyz.invoke(ptr));
    }

    public Vector3 ypr() {
        return new Vector3((MemorySegment) FF.Rot3_ypr.invoke(ptr));
    }

    public Vector3 rpy() {
        return new Vector3((MemorySegment) FF.Rot3_rpy.invoke(ptr));
    }

    public Vector3 xyz(Matrix H) {
        return new Vector3((MemorySegment) FF.Rot3_xyzH.invoke(ptr, H.ptr));
    }

    public Vector3 ypr(Matrix H) {
        return new Vector3((MemorySegment) FF.Rot3_yprH.invoke(ptr, H.ptr));
    }

    public Vector3 rpy(Matrix H) {
        return new Vector3((MemorySegment) FF.Rot3_rpyH.invoke(ptr, H.ptr));
    }

    public static Pair<Matrix3, Vector3> RQ(Matrix3 A) {
        MemorySegment resultPair = (MemorySegment) FF.Rot3_RQ.invoke(
                (SegmentAllocator) Lib.arena, A.ptr);
        MemorySegment firstPtr = (MemorySegment) Pairs.PtrPair_first.get(resultPair, 0);
        MemorySegment secondPtr = (MemorySegment) Pairs.PtrPair_second.get(resultPair, 0);
        return new Pair<>(new Matrix3(firstPtr), new Vector3(secondPtr));
    }

    public static Pair<Matrix3, Vector3> RQ(Matrix3 A, Matrix H) {
        MemorySegment resultPair = (MemorySegment) FF.Rot3_RQH.invoke(
                (SegmentAllocator) Lib.arena, A.ptr, H.ptr);
        MemorySegment firstPtr = (MemorySegment) Pairs.PtrPair_first.get(resultPair, 0);
        MemorySegment secondPtr = (MemorySegment) Pairs.PtrPair_second.get(resultPair, 0);
        return new Pair<>(new Matrix3(firstPtr), new Vector3(secondPtr));
    }

    public Quaternion toQuaternion() {
        return new Quaternion((MemorySegment) FF.Rot3_toQuaternion.invoke(ptr));
    }

    public Rot3 slerp(double t, Rot3 other) {
        return new Rot3((MemorySegment) FF.Rot3_slerp.invoke(ptr, t, other.ptr));
    }

    public static Rot3 interpolate(Rot3 X, Rot3 Y, double t) {
        return new Rot3((MemorySegment) FF.Rot3_interpolate.invoke(X.ptr, Y.ptr, t));
    }
}
