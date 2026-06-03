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
        Pose2_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Pose2_Adjoint(ADDRESS, ADDRESS, ADDRESS),
        Pose2_Expmap(ADDRESS, ADDRESS),
        Pose2_ExpmapH(ADDRESS, ADDRESS, ADDRESS),
        Pose2_Logmap(ADDRESS, ADDRESS),
        Pose2_LogmapH(ADDRESS, ADDRESS, ADDRESS),
        Pose2_logmap(ADDRESS, ADDRESS, ADDRESS),
        Pose2_OriginRetract(ADDRESS, ADDRESS),
        Pose2_OriginLocalCoordinates(ADDRESS, ADDRESS),
        Pose2_OriginRetractH(ADDRESS, ADDRESS, ADDRESS),
        Pose2_OriginLocalCoordinatesH(ADDRESS, ADDRESS, ADDRESS),
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
        Pose2_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Pose2Traits implements LieGroup.Traits<Pose2, Vector3> {

        @Override
        public Pose2 Identity() throws Throwable {
            return statics.Identity();
        }

        @Override
        public Vector3 Logmap(Pose2 g) throws Throwable {
            return statics.Logmap(g);
        }

        @Override
        public Vector3 Logmap(Pose2 g, Matrix H) throws Throwable {
            return statics.Logmap(g, H);
        }

        @Override
        public Pose2 Expmap(Vector3 xi) throws Throwable {
            return statics.Expmap(xi);
        }

        @Override
        public Pose2 Expmap(Vector3 xi, Matrix H) throws Throwable {
            return statics.Expmap(xi, H);
        }

    }

    public static final Pose2Traits traits = new Pose2Traits();

    @Override
    public Pose2Traits traits() {
        return traits;
    }

    public static class Pose2Statics implements LieGroup.Statics<Pose2, Vector3> {
        @Override
        public Pose2 Identity() throws Throwable {
            return new Pose2();
        }

        @Override
        public Vector3 Logmap(Pose2 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Pose2_Logmap.h.invokeExact(g.ptr));
        }

        @Override
        public Vector3 Logmap(Pose2 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Pose2_LogmapH.h.invokeExact(
                    g.ptr, H.ptr));
        }

        @Override
        public Pose2 Expmap(Vector3 xi) throws Throwable {
            return new Pose2((MemorySegment) FF.Pose2_Expmap.h.invokeExact(xi.ptr));
        }

        @Override
        public Pose2 Expmap(Vector3 xi, Matrix H) throws Throwable {
            return new Pose2((MemorySegment) FF.Pose2_ExpmapH.h.invokeExact(
                    xi.ptr, H.ptr));
        }

        @Override
        public Pose2 Retract(Vector3 v) throws Throwable {
            return new Pose2((MemorySegment) FF.Pose2_OriginRetract.h.invokeExact(v.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Pose2 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Pose2_OriginLocalCoordinates.h.invokeExact(g.ptr));
        }

        @Override
        public Pose2 Retract(Vector3 v, Matrix H) throws Throwable {
            return new Pose2((MemorySegment) FF.Pose2_OriginRetractH.h.invokeExact(v.ptr, H.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Pose2 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Pose2_OriginLocalCoordinatesH.h.invokeExact(g.ptr, H.ptr));
        }

    }

    public static final Pose2Statics statics = new Pose2Statics();

    @Override
    public Pose2Statics statics() {
        return statics;
    }

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3(0, 0, 0);
    }

    public Pose2(MemorySegment p) {
        super(p, FF.Pose2_delete.h);
    }

    public Pose2() throws Throwable {
        this(0, 0, 0);
    }

    public Pose2(double x, double y, double theta) throws Throwable {
        this((MemorySegment) FF.Pose2.h.invokeExact(x, y, theta));
    }

    public Pose2(double theta, Point2 t) throws Throwable {
        this((MemorySegment) FF.Pose2DoublePoint2.h.invokeExact(theta, t.ptr));
    }

    /** Copies the arguments. */
    public Pose2(Rot2 r, Point2 t) throws Throwable {
        this((MemorySegment) FF.Pose2Rot2Point2.h.invokeExact(r.ptr, t.ptr));
    }

    public Pose2(Matrix3 T) throws Throwable {
        this((MemorySegment) FF.Pose2Matrix3.h.invokeExact(T.ptr));
    }

    public Pose2(Vector3 v) throws Throwable {
        this((MemorySegment) FF.Pose2Vector3.h.invokeExact(v.ptr));
    }

    @Override
    public Pose2 retract(Vector3 v) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_retract.h.invokeExact(ptr, v.ptr()));
    }

    @Override
    public Pose2 retract(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_retractH.h.invokeExact(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    public static Pose2 Retract(Pose2 origin, Vector3 v, Matrix Horigin, Matrix Hv) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_Retract.h.invokeExact(origin.ptr, v.ptr, Horigin.ptr, Hv.ptr));
    }

    public double x() throws Throwable {
        return (double) FF.Pose2_x.h.invokeExact(ptr);
    }

    public double y() throws Throwable {
        return (double) FF.Pose2_y.h.invokeExact(ptr);
    }

    public double theta() throws Throwable {
        return (double) FF.Pose2_theta.h.invokeExact(ptr);
    }

    public Point2 t() throws Throwable {
        return new Point2((MemorySegment) FF.Pose2_t.h.invokeExact(ptr));
    }

    public Rot2 r() throws Throwable {
        return new Rot2((MemorySegment) FF.Pose2_r.h.invokeExact(ptr));
    }

    @Override
    public Vector3 localCoordinates(Pose2 g) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.Pose2_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector3 localCoordinates(Pose2 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.Pose2_localCoordinatesH.h.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Pose2 between(Pose2 other) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_between.h.invokeExact(ptr, other.ptr));
    }

    public Pose2 between(Pose2 other, Matrix H1, Matrix H2) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_betweenH.h.invokeExact(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Pose2 inverse() throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_inverse.h.invokeExact(ptr));
    }

    public Pose2 inverse(Matrix H) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_inverseH.h.invokeExact(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() throws Throwable {
        // coerce Matrix3 to dynamic. */
        return new Matrix((MemorySegment) FF.Pose2_AdjointMap.h.invokeExact(ptr));
    }

    @Override
    public Pose2 expmap(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_expmapH.h.invokeExact(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector3 logmap(Pose2 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3((MemorySegment) FF.Pose2_logmapH.h.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public Vector3 Adjoint(Vector3 v) throws Throwable {
        return new Vector3((MemorySegment) FF.Pose2_Adjoint.h.invokeExact(ptr, v.ptr));
    }

    public static Pose2 Expmap(Vector3 xi, Matrix Hv) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_ExpmapH.h.invokeExact(xi.ptr, Hv.ptr));
    }

    public static Vector3 Logmap(Pose2 p, Matrix H) throws Throwable {
        return new Vector3((MemorySegment) FF.Pose2_LogmapH.h.invokeExact(p.ptr, H.ptr));
    }

    public Vector3 logmap(Pose2 p) throws Throwable {
        return new Vector3((MemorySegment) FF.Pose2_logmap.h.invokeExact(ptr, p.ptr));
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
        return new Pose2((MemorySegment) FF.Pose2_compose.h.invokeExact(ptr, other.ptr));
    }

    public Pose2 compose(Pose2 other, Matrix H1, Matrix H2) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_composeH.h.invokeExact(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    public Matrix3 matrix() throws Throwable {
        return new Matrix3((MemorySegment) FF.Pose2_matrix.h.invokeExact(ptr));
    }

    // TODO: maybe make this static?
    public Vector3 logmap_default(Pose2 p) throws Throwable {
        return new Vector3((MemorySegment) FF.Pose2_logmap_default.h.invokeExact(ptr, p.ptr));
    }

    // TODO: maybe make this static?
    public Pose2 expmap_default(Vector3 d) throws Throwable {
        return new Pose2((MemorySegment) FF.Pose2_expmap_default.h.invokeExact(ptr, d.ptr));
    }

    public Point2 transformTo(Point2 point) throws Throwable {
        return new Point2((MemorySegment) FF.Pose2_transformTo.h.invokeExact(
                ptr, point.ptr));
    }

    public Point2 transformTo(//
            Point2 point, //
            Matrix Dpose, //
            Matrix Dpoint) throws Throwable {
        return new Point2((MemorySegment) FF.Pose2_transformToH.h.invokeExact(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public Point2 transformFrom(Point2 point) throws Throwable {
        return new Point2((MemorySegment) FF.Pose2_transformFrom.h.invokeExact(
                ptr, point.ptr));
    }

    public Point2 transformFrom(//
            Point2 point, //
            Matrix Dpose, //
            Matrix Dpoint) throws Throwable {
        return new Point2((MemorySegment) FF.Pose2_transformFromH.h.invokeExact(
                ptr, point.ptr, Dpose.ptr, Dpoint.ptr));
    }

    public static Matrix3 ExpmapDerivative(Vector3 v) throws Throwable {
        return new Matrix3((MemorySegment) FF.Pose2_ExpmapDerivative.h.invokeExact(v.ptr));
    }

    public Point2 translation(Matrix H) throws Throwable {
        return new Point2((MemorySegment) FF.Pose2_translation.h.invokeExact(ptr, H.ptr));
    }

    public Rot2 bearing(Point2 p) throws Throwable {
        return new Rot2((MemorySegment) FF.Pose2_bearingPoint2.h.invokeExact(ptr, p.ptr));
    }

    public Rot2 bearing(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) FF.Pose2_bearingPoint2H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Rot2 bearing(Pose2 p) throws Throwable {
        return new Rot2((MemorySegment) FF.Pose2_bearingPose2.h.invokeExact(ptr, p.ptr));
    }

    public Rot2 bearing(Pose2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) FF.Pose2_bearingPose2H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public double range(Point2 p) throws Throwable {
        return (double) FF.Pose2_rangePoint2.h.invokeExact(ptr, p.ptr);
    }

    public double range(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return (double) FF.Pose2_rangePoint2H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr);
    }

    public double range(Pose2 p) throws Throwable {
        return (double) FF.Pose2_rangePose2.h.invokeExact(ptr, p.ptr);
    }

    public double range(Pose2 p, Matrix H1, Matrix H2) throws Throwable {
        return (double) FF.Pose2_rangePose2H.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr);
    }

    public static boolean check_group_invariants(Pose2 a, Pose2 b) throws Throwable {
        return (boolean) FF.Pose2_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Pose2 a, Pose2 b) throws Throwable {
        return (boolean) FF.Pose2_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

}
