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
        // Rot3_logmap(ADDRESS, ADDRESS, ADDRESS),
        // Rot3_expmap(ADDRESS, ADDRESS, ADDRESS),
        Rot3_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_OriginRetract(ADDRESS, ADDRESS),
        Rot3_OriginLocalCoordinates(ADDRESS, ADDRESS),
        Rot3_OriginRetractH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_OriginLocalCoordinatesH(ADDRESS, ADDRESS, ADDRESS),
        Rot3_Expmap(ADDRESS, ADDRESS),
        Rot3_ExpmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot3_Logmap(ADDRESS, ADDRESS),
        Rot3_LogmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
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
        Rot3_toQuaternion(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        FF(StructLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Rot3Traits implements LieGroup.Traits<Rot3, Vector3> {

        @Override
        public Rot3 Identity() throws Throwable {
            return statics.Identity();
        }

        @Override
        public Vector3 Logmap(Rot3 g) throws Throwable {
            return statics.Logmap(g);
        }

        @Override
        public Vector3 Logmap(Rot3 g, Matrix H) throws Throwable {
            return statics.Logmap(g, H);
        }

        @Override
        public Rot3 Expmap(Vector3 v) throws Throwable {
            return statics.Expmap(v);
        }

        @Override
        public Rot3 Expmap(Vector3 v, Matrix H) throws Throwable {
            return statics.Expmap(v, H);
        }

    }

    public static final Rot3Traits traits = new Rot3Traits();

    @Override
    public Traits<Rot3, Vector3> traits() {
        return traits;
    }

    public static class Rot3Statics implements LieGroup.Statics<Rot3, Vector3> {
        @Override
        public Rot3 Identity() throws Throwable {
            return new Rot3();
        }

        @Override
        public Vector3 Logmap(Rot3 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_Logmap.h.invokeExact(g.ptr));
        }

        @Override
        public Vector3 Logmap(Rot3 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_LogmapH.h.invokeExact(
                    g.ptr, H.ptr));
        }

        @Override
        public Rot3 Expmap(Vector3 v) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_Expmap.h.invokeExact(v.ptr));
        }

        @Override
        public Rot3 Expmap(Vector3 v, Matrix H) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_ExpmapH.h.invokeExact(
                    v.ptr, H.ptr));
        }

        @Override
        public Rot3 Retract(Vector3 v) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_OriginRetract.h.invokeExact(v.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Rot3 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_OriginLocalCoordinates.h.invokeExact(g.ptr));
        }

        @Override
        public Rot3 Retract(Vector3 v, Matrix H) throws Throwable {
            return new Rot3((MemorySegment) FF.Rot3_OriginRetractH.h.invokeExact(v.ptr, H.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Rot3 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Rot3_OriginLocalCoordinatesH.h.invokeExact(g.ptr, H.ptr));
        }
    }

    public static final Rot3Statics statics = new Rot3Statics();

    @Override
    public Rot3Statics statics() {
        return statics;
    }

    public Rot3(MemorySegment p) {
        super(p, FF.Rot3_delete.h);
    }

    public Rot3(Point3 col1, Point3 col2, Point3 col3) throws Throwable {
        this((MemorySegment) FF.Rot3Point3.h.invokeExact(
                col1.ptr, col2.ptr, col3.ptr));
    }

    /** Identity. */
    public Rot3() throws Throwable {
        this(Matrix3.identity());
    }

    public Rot3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) FF.Rot3.h.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public Rot3(Matrix3 R) throws Throwable {
        this((MemorySegment) FF.Rot3Matrix3.h.invokeExact(R.ptr));
    }

    public Rot3(Quaternion q) throws Throwable {
        this((MemorySegment) FF.Rot3Quaternion.h.invokeExact(q.ptr));
    }

    public static Rot3 Quaternion(double w, double x, double y, double z) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Quaternion.h.invokeExact(w, x, y, z));
    }

    public static Rot3 Yaw(double t) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Yaw.h.invokeExact(t));
    }

    public static Rot3 Pitch(double t) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Pitch.h.invokeExact(t));
    }

    public static Rot3 Roll(double t) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Roll.h.invokeExact(t));
    }

    public static Rot3 Ypr(double y, double p, double r) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Ypr.h.invokeExact(y, p, r));
    }

    public static Rot3 Ypr(double y, double p, double r, //
            Matrix H1, Matrix H2, Matrix H3) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_YprH.h.invokeExact(y, p, r, H1.ptr, H2.ptr, H3.ptr));
    }

    public static Rot3 Rodrigues(double wx, double wy, double wz) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Rodrigues.h.invokeExact(wx, wy, wz));
    }

    public static Rot3 Rodrigues(Point3 v) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_RodriguesVector3.h.invokeExact(v.ptr));
    }

    public static Rot3 AxisAngle(Point3 axis, double angle) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_AxisAnglePoint3.h.invokeExact(axis.ptr, angle));
    }

    public static Rot3 AxisAngle(Unit3 axis, double angle) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_AxisAngleUnit3.h.invokeExact(axis.ptr, angle));
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
        return new Vector3((MemorySegment) FF.Rot3_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector3 localCoordinates(Rot3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_localCoordinatesH.h.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_retract.h.invokeExact(ptr, v.ptr));
    }

    @Override
    public Rot3 retract(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_retract.h.invokeExact(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    // public Vector3 logmap(Rot3 g) throws Throwable {
    // return new Vector3((MemorySegment) FF.Rot3_logmap.h.invokeExact(ptr, g.ptr));
    // }

    // public Rot3 expmap(Vector3 v) throws Throwable {
    // return new Rot3((MemorySegment) FF.Rot3_expmap.h.invokeExact(ptr, v.ptr));
    // }

    @Override
    public Rot3 expmap(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_expmapH.h.invokeExact(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector3 logmap(Rot3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_logmapH.h.invokeExact(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Matrix3 matrix() throws Throwable {
        return new Matrix3((MemorySegment) FF.Rot3_matrix.h.invokeExact(ptr));
    }

    @Override
    public Rot3 compose(Rot3 other) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_compose.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Rot3 compose(Rot3 other, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_composeH.h.invokeExact(
                ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 between(Rot3 g) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_between.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Rot3 between(Rot3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_betweenH.h.invokeExact(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot3 inverse() throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_inverse.h.invokeExact(ptr));
    }

    @Override
    public Rot3 inverse(Matrix H) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_inverseH.h.invokeExact(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) FF.Rot3_AdjointMap.h.invokeExact(ptr));
    }

    public Matrix3 transpose() throws Throwable {
        return new Matrix3((MemorySegment) FF.Rot3_transpose.h.invokeExact(ptr));
    }

    public Unit3 rotate(Unit3 p) throws Throwable {
        return new Unit3((MemorySegment) FF.Rot3_rotateUnit3.h.invokeExact(ptr, p.ptr));
    }

    public Point3 rotate(Point3 p) throws Throwable {
        return new Point3((MemorySegment) FF.Rot3_rotatePoint3.h.invokeExact(ptr, p.ptr));
    }

    public Unit3 rotate(Unit3 p, Matrix H1, Matrix H2) throws Throwable {
        return new Unit3((MemorySegment) FF.Rot3_rotateUnit3H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point3 rotate(Point3 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Rot3_rotatePoint3H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Unit3 unrotate(Unit3 p) throws Throwable {
        return new Unit3((MemorySegment) FF.Rot3_unrotateUnit3.h.invokeExact(ptr, p.ptr));
    }

    public Point3 unrotate(Point3 p) throws Throwable {
        return new Point3((MemorySegment) FF.Rot3_unrotatePoint3.h.invokeExact(ptr, p.ptr));
    }

    public Unit3 unrotate(Unit3 p, Matrix H1, Matrix H2) throws Throwable {
        return new Unit3((MemorySegment) FF.Rot3_unrotateUnit3H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point3 unrotate(Point3 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Rot3_unrotatePoint3H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public static boolean check_group_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) FF.Rot3_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot3 a, Rot3 b) throws Throwable {
        return (boolean) FF.Rot3_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public Pair<Unit3, Double> axisAngle() throws Throwable {
        MemorySegment resultStruct = (MemorySegment) FF.Rot3_axisAngle.h.invokeExact(
                (SegmentAllocator) Lib.arena, ptr);
        MemorySegment first = (MemorySegment) AxisAngle_first.get(resultStruct, 0);
        double second = (double) AxisAngle_second.get(resultStruct, 0);
        return new Pair<>(new Unit3(first), second);
    }

    public static Rot3 ClosestTo(Matrix3 M) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_ClosestTo.h.invokeExact(M.ptr));
    }

    public static Rot3 Rx(double t) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Rx.h.invokeExact(t));
    }

    public static Rot3 Ry(double t) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Ry.h.invokeExact(t));
    }

    public static Rot3 Rz(double t) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_Rz.h.invokeExact(t));
    }

    public static Rot3 RzRyRx(double x, double y, double z) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRx.h.invokeExact(x, y, z));
    }

    public static Rot3 RzRyRx(double x, double y, double z, //
            Matrix H1, Matrix H2, Matrix H3) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRxH.h.invokeExact(x, y, z, H1.ptr, H2.ptr, H3.ptr));
    }

    public static Rot3 RzRyRx(Vector3 xyz) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRxVector3.h.invokeExact(xyz.ptr));
    }

    public static Rot3 RzRyRx(Vector3 xyz, Matrix H) throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_RzRyRxVector3H.h.invokeExact(xyz.ptr, H.ptr));
    }

    public Rot3 normalized() throws Throwable {
        return new Rot3((MemorySegment) FF.Rot3_normalized.h.invokeExact(ptr));
    }

    public double roll() throws Throwable {
        return (double) FF.Rot3_roll.h.invokeExact(ptr);
    }

    public double pitch() throws Throwable {
        return (double) FF.Rot3_pitch.h.invokeExact(ptr);
    }

    public double yaw() throws Throwable {
        return (double) FF.Rot3_yaw.h.invokeExact(ptr);
    }

    public double roll(Matrix H) throws Throwable {
        return (double) FF.Rot3_rollH.h.invokeExact(ptr, H.ptr);
    }

    public double pitch(Matrix H) throws Throwable {
        return (double) FF.Rot3_pitchH.h.invokeExact(ptr, H.ptr);
    }

    public double yaw(Matrix H) throws Throwable {
        return (double) FF.Rot3_yawH.h.invokeExact(ptr, H.ptr);
    }

    public Vector3 xyz() throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_xyz.h.invokeExact(ptr));
    }

    public Vector3 ypr() throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_ypr.h.invokeExact(ptr));
    }

    public Vector3 rpy() throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_rpy.h.invokeExact(ptr));
    }

    public Vector3 xyz(Matrix H) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_xyzH.h.invokeExact(ptr, H.ptr));
    }

    public Vector3 ypr(Matrix H) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_yprH.h.invokeExact(ptr, H.ptr));
    }

    public Vector3 rpy(Matrix H) throws Throwable {
        return new Vector3((MemorySegment) FF.Rot3_rpyH.h.invokeExact(ptr, H.ptr));
    }

    public static Pair<Matrix3, Vector3> RQ(Matrix3 A) throws Throwable {
        MemorySegment resultPair = (MemorySegment) FF.Rot3_RQ.h.invokeExact(
                (SegmentAllocator) Lib.arena, A.ptr);
        MemorySegment firstPtr = (MemorySegment) Pairs.PtrPair_first.get(resultPair, 0);
        MemorySegment secondPtr = (MemorySegment) Pairs.PtrPair_second.get(resultPair, 0);
        return new Pair<>(new Matrix3(firstPtr), new Vector3(secondPtr));
    }

    public static Pair<Matrix3, Vector3> RQ(Matrix3 A, Matrix H) throws Throwable {
        MemorySegment resultPair = (MemorySegment) FF.Rot3_RQH.h.invokeExact(
                (SegmentAllocator) Lib.arena, A.ptr, H.ptr);
        MemorySegment firstPtr = (MemorySegment) Pairs.PtrPair_first.get(resultPair, 0);
        MemorySegment secondPtr = (MemorySegment) Pairs.PtrPair_second.get(resultPair, 0);
        return new Pair<>(new Matrix3(firstPtr), new Vector3(secondPtr));
    }

    public Quaternion toQuaternion() throws Throwable {
        return new Quaternion((MemorySegment) FF.Rot3_toQuaternion.h.invokeExact(ptr));
    }
}
