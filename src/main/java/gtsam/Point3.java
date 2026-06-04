package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * TODO: Point3 is a typedef of Vector3, so this seems wrong.
 * TODO: implement vectortype
 */
public class Point3 extends ForeignObject
        implements LieGroup<Point3, Vector3> {
    public enum FF {
        Point3(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Point3_delete(null, ADDRESS),
        Point3_x(JAVA_DOUBLE, ADDRESS),
        Point3_y(JAVA_DOUBLE, ADDRESS),
        Point3_z(JAVA_DOUBLE, ADDRESS),
        Point3_plus(ADDRESS, ADDRESS, ADDRESS),
        Point3_minus(ADDRESS, ADDRESS, ADDRESS),
        Point3_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Point3_cross(ADDRESS, ADDRESS, ADDRESS),
        Point3_crossPoint3Point3(ADDRESS, ADDRESS, ADDRESS),
        Point3_crossPoint3Point3H(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point3_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point3_Logmap(ADDRESS, ADDRESS),
        Point3_Expmap(ADDRESS, ADDRESS),
        Point3_LogmapH(ADDRESS, ADDRESS, ADDRESS),
        Point3_ExpmapH(ADDRESS, ADDRESS, ADDRESS),
        Point3_compose(ADDRESS, ADDRESS, ADDRESS),
        Point3_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_between(ADDRESS, ADDRESS, ADDRESS),
        Point3_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_inverse(ADDRESS, ADDRESS),
        Point3_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Point3_AdjointMap(ADDRESS, ADDRESS),
        Point3_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_retract(ADDRESS, ADDRESS, ADDRESS),
        Point3_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_OriginRetract(ADDRESS, ADDRESS),
        Point3_OriginLocalCoordinates(ADDRESS, ADDRESS),
        Point3_OriginRetractH(ADDRESS, ADDRESS, ADDRESS),
        Point3_OriginLocalCoordinatesH(ADDRESS, ADDRESS, ADDRESS),
        Point3_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Point3_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_dot(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Point3_dotPoint3Point3(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Point3_dotPoint3Point3H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point3_normalize(ADDRESS, ADDRESS),
        Point3_normalizeH(ADDRESS, ADDRESS, ADDRESS),
        Point3_norm3(JAVA_DOUBLE, ADDRESS),
        Point3_norm3H(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Point3_norm(JAVA_DOUBLE, ADDRESS),
        Point3_distance3(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Point3_distance3H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    Point3(MemorySegment p) {
        super(p, FF.Point3_delete.h);
    }

    public Point3(double x, double y, double z) throws Throwable {
        this((MemorySegment) FF.Point3.h.invokeExact(x, y, z));
    }

    public Point3(Vector3 v) throws Throwable {
        this(v.at(0), v.at(1), v.at(2));
    }

    public double x() throws Throwable {
        return (double) FF.Point3_x.h.invokeExact(ptr);
    }

    public double y() throws Throwable {
        return (double) FF.Point3_y.h.invokeExact(ptr);
    }

    public double z() throws Throwable {
        return (double) FF.Point3_z.h.invokeExact(ptr);
    }

    public Point3 plus(Point3 other) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_plus.h.invokeExact(ptr, other.ptr));
    }

    public Point3 minus(Point3 other) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_minus.h.invokeExact(ptr, other.ptr));
    }

    public Point3 times(double a) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_times.h.invokeExact(ptr, a));
    }

    public static boolean check_group_invariants(Point3 a, Point3 b) throws Throwable {
        return (boolean) FF.Point3_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Point3 a, Point3 b) throws Throwable {
        return (boolean) FF.Point3_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static class Traits implements LieGroup.Traits<Point3, Vector3> {

        @Override
        public Point3 Identity() throws Throwable {
            return statics.Identity();
        }

        @Override
        public Vector3 Logmap(Point3 g) throws Throwable {
            return statics.Logmap(g);
        }

        @Override
        public Vector3 Logmap(Point3 g, Matrix H) throws Throwable {
            return statics.Logmap(g, H);
        }

        @Override
        public Point3 Expmap(Vector3 v) throws Throwable {
            return statics.Expmap(v);
        }

        @Override
        public Point3 Expmap(Vector3 v, Matrix H) throws Throwable {
            return statics.Expmap(v, H);
        }

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3(0, 0, 0);
    }

    @Override
    public int dimension() throws Throwable {
        return 3;
    }

    public Point3 cross(Point3 q) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_cross.h.invokeExact(ptr, q.ptr));
    }

    public static Point3 cross(Point3 p, Point3 q) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_crossPoint3Point3.h.invokeExact(p.ptr, q.ptr));
    }

    public static Point3 cross(Point3 p, Point3 q, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_crossPoint3Point3H.h.invokeExact(p.ptr, q.ptr, H1.ptr, H2.ptr));
    }

    public static double dot(Point3 p, Point3 q) throws Throwable {
        return (double) FF.Point3_dotPoint3Point3.h.invokeExact(p.ptr, q.ptr);
    }

    public static double dot(Point3 p, Point3 q, Matrix H1, Matrix H2) throws Throwable {
        return (double) FF.Point3_dotPoint3Point3H.h.invokeExact(p.ptr, q.ptr, H1.ptr, H2.ptr);
    }

    public static class Statics implements LieGroup.Statics<Point3, Vector3> {

        @Override
        public Point3 Identity() throws Throwable {
            return new Point3(0, 0, 0);
        }

        @Override
        public Vector3 Logmap(Point3 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Point3_Logmap.h.invokeExact(g.ptr));
        }

        @Override
        public Vector3 Logmap(Point3 m, Matrix Hm) throws Throwable {
            return new Vector3((MemorySegment) FF.Point3_LogmapH.h.invokeExact(m.ptr, Hm.ptr));
        }

        @Override
        public Point3 Expmap(Vector3 v) throws Throwable {
            return new Point3((MemorySegment) FF.Point3_Expmap.h.invokeExact(v.ptr));
        }

        @Override
        public Point3 Expmap(Vector3 v, Matrix Hv) throws Throwable {
            return new Point3((MemorySegment) FF.Point3_ExpmapH.h.invokeExact(v.ptr, Hv.ptr));
        }

        @Override
        public Point3 Retract(Vector3 v) throws Throwable {
            return new Point3((MemorySegment) FF.Point3_OriginRetract.h.invokeExact(v.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Point3 g) throws Throwable {
            return new Vector3((MemorySegment) FF.Point3_OriginLocalCoordinates.h.invokeExact(g.ptr));
        }

        @Override
        public Point3 Retract(Vector3 v, Matrix H) throws Throwable {
            return new Point3((MemorySegment) FF.Point3_OriginRetractH.h.invokeExact(v.ptr, H.ptr));
        }

        @Override
        public Vector3 LocalCoordinates(Point3 g, Matrix H) throws Throwable {
            return new Vector3((MemorySegment) FF.Point3_OriginLocalCoordinatesH.h.invokeExact(g.ptr, H.ptr));
        }
    }

    public static final Statics statics = new Statics();

    @Override
    public Statics statics() {
        return statics;
    }

    @Override
    public Point3 compose(Point3 h) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_compose.h.invokeExact(ptr, h.ptr));
    }

    @Override
    public Point3 compose(Point3 h, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_composeH.h.invokeExact(
                ptr, h.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Point3 between(Point3 h) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_between.h.invokeExact(ptr, h.ptr));

    }

    @Override
    public Point3 between(Point3 h, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_betweenH.h.invokeExact(
                ptr, h.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Point3 inverse() throws Throwable {
        return new Point3((MemorySegment) FF.Point3_inverse.h.invokeExact(ptr));
    }

    @Override
    public Point3 inverse(Matrix H) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_inverseH.h.invokeExact(ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() throws Throwable {
        return new Matrix((MemorySegment) FF.Point3_AdjointMap.h.invokeExact(ptr));
    }

    @Override
    public Point3 expmap(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_expmapH.h.invokeExact(ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector3 logmap(Point3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3((MemorySegment) FF.Point3_logmapH.h.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Point3 retract(Vector3 v) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_retract.h.invokeExact(ptr, v.ptr()));
    }

    @Override
    public Point3 retract(Vector3 v, Matrix H1, Matrix H2) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_retractH.h.invokeExact(
                ptr, v.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Vector3 localCoordinates(Point3 g) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.Point3_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    @Override
    public Vector3 localCoordinates(Point3 g, Matrix H1, Matrix H2) throws Throwable {
        return new Vector3(
                (MemorySegment) FF.Point3_localCoordinatesH.h.invokeExact(ptr, g.ptr, H1.ptr, H2.ptr));
    }

    public double dot(Point3 g) throws Throwable {
        return (double) FF.Point3_dot.h.invokeExact(ptr, g.ptr);
    }

    public static Point3 normalize(Point3 p) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_normalize.h.invokeExact(p.ptr));
    }

    public static Point3 normalize(Point3 p, Matrix H) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_normalizeH.h.invokeExact(p.ptr, H.ptr));
    }

    public static double norm3(Point3 p) throws Throwable {
        return (double) FF.Point3_norm3.h.invokeExact(p.ptr);
    }

    public static double norm3(Point3 p, Matrix H) throws Throwable {
        return (double) FF.Point3_norm3H.h.invokeExact(p.ptr, H.ptr);
    }

    public double norm() throws Throwable {
        // This is the Eigen norm.
        return (double) FF.Point3_norm.h.invokeExact(ptr);
    }

    public static double distance3(Point3 p, Point3 q) throws Throwable {
        return (double) FF.Point3_distance3.h.invokeExact(p.ptr, q.ptr);
    }

    public static double distance3(Point3 p, Point3 q, Matrix H1, Matrix H2) throws Throwable {
        return (double) FF.Point3_distance3H.h.invokeExact(p.ptr, q.ptr, H1.ptr, H2.ptr);
    }
}
