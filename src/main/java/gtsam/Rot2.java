package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot2 extends ForeignObject implements LieGroup<Rot2, Vector1> {
    public enum FF {
        Rot2(ADDRESS, JAVA_DOUBLE),
        Rot2_delete(null, ADDRESS),
        Rot2_theta(JAVA_DOUBLE, ADDRESS),
        Rot2_c(JAVA_DOUBLE, ADDRESS),
        Rot2_s(JAVA_DOUBLE, ADDRESS),
        Rot2_matrix(ADDRESS, ADDRESS),
        Rot2_compose(ADDRESS, ADDRESS, ADDRESS),
        Rot2_composeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_rotate(ADDRESS, ADDRESS, ADDRESS),
        Rot2_rotateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_unrotate(ADDRESS, ADDRESS, ADDRESS),
        Rot2_unrotateH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_fromCosSin(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot2_atan2(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        Rot2_relativeBearing(ADDRESS, ADDRESS),
        Rot2_relativeBearingH(ADDRESS, ADDRESS, ADDRESS),
        Rot2_unit(ADDRESS, ADDRESS),
        Rot2_inverse(ADDRESS, ADDRESS),
        Rot2_inverseH(ADDRESS, ADDRESS, ADDRESS),
        Rot2_expmap(ADDRESS, ADDRESS, ADDRESS),
        Rot2_expmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_logmap(ADDRESS, ADDRESS, ADDRESS),
        Rot2_logmapH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_transpose(ADDRESS, ADDRESS),
        Rot2_between(ADDRESS, ADDRESS, ADDRESS),
        Rot2_betweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_retract(ADDRESS, ADDRESS, ADDRESS),
        Rot2_retractH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_localCoordinates(ADDRESS, ADDRESS, ADDRESS),
        Rot2_localCoordinatesH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Rot2_Expmap(ADDRESS, ADDRESS),
        Rot2_Logmap(ADDRESS, ADDRESS),
        Rot2_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Rot2_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Rot2Traits implements LieGroup.Traits<Rot2, Vector1> {

        @Override
        public Rot2 Identity() throws Throwable {
            return new Rot2();
        }

        @Override
        public Rot2 Expmap(Vector1 v) throws Throwable {
            return new Rot2((MemorySegment) FF.Rot2_Expmap.h.invokeExact(v.ptr));
        }

        @Override
        public Vector1 Logmap(Rot2 g) throws Throwable {
            return new Vector1((MemorySegment) FF.Rot2_Logmap.h.invokeExact(g.ptr));
        }
    }

    // public static class Rot2Statics implements LieGroup.Statics<Rot2, Vector1> {
    // @Override
    // public Rot2 Identity() throws Throwable {
    // return new Rot2();
    // }
    // }

    public static final Rot2Traits traits = new Rot2Traits();

    @Override
    public Traits<Rot2, Vector1> traits() {
        return traits;
    }

    public Rot2(MemorySegment p) {
        super(p, FF.Rot2_delete.h);
    }

    public Rot2() throws Throwable {
        this(0);
    }

    public Rot2(double theta) throws Throwable {
        // this((MemorySegment) Rot2.invokeExact(theta));
        this((MemorySegment) FF.Rot2.h.invokeExact(theta));
    }

    public static Rot2 fromAngle(double theta) throws Throwable {
        return new Rot2(theta);
    }

    public double theta() throws Throwable {
        return (double) FF.Rot2_theta.h.invokeExact(ptr);
    }

    public double c() throws Throwable {
        return (double) FF.Rot2_c.h.invokeExact(ptr);
    }

    public double s() throws Throwable {
        return (double) FF.Rot2_s.h.invokeExact(ptr);
    }

    public Matrix2 matrix() throws Throwable {
        return new Matrix2((MemorySegment) FF.Rot2_matrix.h.invokeExact(ptr));
    }

    @Override
    public Rot2 compose(Rot2 other) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_compose.h.invokeExact(ptr, other.ptr));
    }

    public Rot2 compose(Rot2 other, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_composeH.h.invokeExact(ptr, other.ptr, H1.ptr, H2.ptr));
    }

    public Point2 rotate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) FF.Rot2_rotate.h.invokeExact(ptr, p.ptr));
    }

    public Point2 rotate(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point2((MemorySegment) FF.Rot2_rotateH.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public Point2 unrotate(Point2 p) throws Throwable {
        return new Point2((MemorySegment) FF.Rot2_unrotate.h.invokeExact(ptr, p.ptr));
    }

    public Point2 unrotate(Point2 p, Matrix H1, Matrix H2) throws Throwable {
        return new Point2((MemorySegment) FF.Rot2_unrotateH.h.invokeExact(ptr, p.ptr, H1.ptr, H2.ptr));
    }

    public static Rot2 fromCosSin(double c, double s) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_fromCosSin.h.invokeExact(c, s));
    }

    /** Note order of arguments, y first, x second. */
    public static Rot2 atan2(double y, double x) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_atan2.h.invokeExact(y, x));
    }

    public static Rot2 relativeBearing(Point2 d) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_relativeBearing.h.invokeExact(d.ptr));
    }

    public static Rot2 relativeBearing(Point2 d, Matrix H) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_relativeBearingH.h.invokeExact(d.ptr, H.ptr));
    }

    public Point2 unit() throws Throwable {
        return new Point2((MemorySegment) FF.Rot2_unit.h.invokeExact(ptr));
    }

    @Override
    public Rot2 inverse() throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_inverse.h.invokeExact(ptr));
    }

    @Override
    public Rot2 inverse(Matrix H) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_inverseH.h.invokeExact(ptr, H.ptr));
    }

    // @Override
    // public Rot2 expmap(Vector1 v) throws Throwable {
    // return new Rot2((MemorySegment) Rot2_expmap.invokeExact(ptr, v.ptr));
    // }

    // @Override
    // public Vector1 logmap(Rot2 g) throws Throwable {
    // return new Vector1((MemorySegment) Rot2_logmap.invokeExact(ptr, g.ptr));
    // }

    // @Override
    // public Rot2 expmap(Vector1 v, Matrix H1, Matrix H2) throws Throwable {
    // return new Rot2((MemorySegment) Rot2_expmapH.invokeExact(ptr, v.ptr, H1.ptr,
    // H2.ptr));
    // }

    // @Override
    // public Vector1 logmap(Rot2 g, Matrix H1, Matrix H2) throws Throwable {
    // return new Vector1((MemorySegment) Rot2_logmapH.invokeExact(ptr, g.ptr,
    // H1.ptr, H2.ptr));
    // }

    public Matrix2 transpose() throws Throwable {
        return new Matrix2((MemorySegment) FF.Rot2_transpose.h.invokeExact(ptr));
    }

    @Override
    public Rot2 between(Rot2 g) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_between.h.invokeExact(ptr, g.ptr));
    }

    // @Override
    public Rot2 between(Rot2 g, Matrix H1, Matrix H2) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_betweenH.h.invokeExact(
                ptr, g.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Rot2 retract(Vector1 v) throws Throwable {
        return new Rot2((MemorySegment) FF.Rot2_retract.h.invokeExact(ptr, v.ptr));
    }

    @Override
    public Vector1 localCoordinates(Rot2 g) throws Throwable {
        return new Vector1((MemorySegment) FF.Rot2_localCoordinates.h.invokeExact(ptr, g.ptr));
    }

    // @Override
    // public Rot2 retract(Vector1 v, Matrix H1, Matrix H2) throws Throwable {
    // return new Rot2((MemorySegment) Rot2_retractH.invokeExact(ptr, v.ptr, H1.ptr,
    // H2.ptr));
    // }

    // @Override
    // public Vector1 localCoordinates(Rot2 g, Matrix H1, Matrix H2) throws
    // Throwable {
    // return new Vector1((MemorySegment) Rot2_localCoordinatesH.invokeExact(ptr,
    // g.ptr, H1.ptr, H2.ptr));
    // }

    @Override
    public Vector1 dxZero() throws Throwable {
        return new Vector1(0);
    }

    @Override
    public int dimension() throws Throwable {
        return 1;
    }

    public static boolean check_group_invariants(Rot2 a, Rot2 b) throws Throwable {
        return (boolean) FF.Rot2_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Rot2 a, Rot2 b) throws Throwable {
        return (boolean) FF.Rot2_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }
}
