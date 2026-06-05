package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

// TODO: implement LieGroup.
// TOOD: implement VectorType
public class Point2 extends ForeignObject
        implements LieGroup<Point2, Vector2> {
    public enum FF {
        Point2(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        Point2_delete(null, ADDRESS),
        Point2_plus(ADDRESS, ADDRESS, ADDRESS),
        Point2_minus(ADDRESS, ADDRESS, ADDRESS),
        Point2_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Point2_x(JAVA_DOUBLE, ADDRESS),
        Point2_y(JAVA_DOUBLE, ADDRESS),
        Point2_norm2(JAVA_DOUBLE, ADDRESS),
        Point2_norm2H(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Point2_distance2(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Point2_distance2H(JAVA_DOUBLE, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point2_norm(JAVA_DOUBLE, ADDRESS),
        Point2_normalized(ADDRESS, ADDRESS),
        Point2_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point2_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point2_Logmap(ADDRESS, ADDRESS),
        Point2_Expmap(ADDRESS, ADDRESS),
        Point2_LogmapH(ADDRESS, ADDRESS, ADDRESS),
        Point2_ExpmapH(ADDRESS, ADDRESS, ADDRESS),
        Point2_Compose(ADDRESS, ADDRESS, ADDRESS),
        Point2_ComposeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point2_Between(ADDRESS, ADDRESS, ADDRESS),
        Point2_BetweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point2_Inverse(ADDRESS, ADDRESS),
        Point2_InverseH(ADDRESS, ADDRESS, ADDRESS),
        Point2_AdjointMap(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Traits implements LieGroup.Traits<Point2, Vector2> {
        @Override
        public Point2 Identity() throws Throwable {
            return statics.Identity();
        }

        @Override
        public Vector2 Logmap(Point2 g) throws Throwable {
            return statics.Logmap(g);
        }

        @Override
        public Vector2 Logmap(Point2 g, Matrix H) throws Throwable {
            return statics.Logmap(g, H);
        }

        @Override
        public Point2 Expmap(Vector2 v) throws Throwable {
            return statics.Expmap(v);
        }

        @Override
        public Point2 Expmap(Vector2 v, Matrix H) throws Throwable {
            return statics.Expmap(v, H);
        }

        @Override
        public Point2 Compose(Point2 m1, Point2 m2) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_Compose.h.invokeExact(m1.ptr, m2.ptr));
        }

        @Override
        public Point2 Compose(Point2 m1, Point2 m2, Matrix H1, Matrix H2) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_ComposeH.h.invokeExact(
                    m1.ptr, m2.ptr, H1.ptr, H2.ptr));
        }

        @Override
        public Point2 Between(Point2 m1, Point2 m2) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_Between.h.invokeExact(m1.ptr, m2.ptr));
        }

        @Override
        public Point2 Between(Point2 m1, Point2 m2, Matrix H1, Matrix H2) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_BetweenH.h.invokeExact(
                    m1.ptr, m2.ptr, H1.ptr, H2.ptr));
        }

        @Override
        public Point2 Inverse(Point2 m1) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_Inverse.h.invokeExact(m1.ptr));
        }

        @Override
        public Point2 Inverse(Point2 m1, Matrix H) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_InverseH.h.invokeExact(
                    m1.ptr, H.ptr));
        }

        @Override
        public Matrix AdjointMap(Point2 m) throws Throwable {
            return new Matrix((MemorySegment) FF.Point2_AdjointMap.h.invokeExact(m.ptr));
        }
    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    public static class Statics implements LieGroup.Statics<Point2, Vector2> {
        @Override
        public Point2 Identity() throws Throwable {
            return new Point2(0, 0);
        }

        @Override
        public Vector2 Logmap(Point2 g) throws Throwable {
            return new Vector2((MemorySegment) FF.Point2_Logmap.h.invokeExact(g.ptr));
        }

        @Override
        public Vector2 Logmap(Point2 m, Matrix Hm) throws Throwable {
            return new Vector2((MemorySegment) FF.Point2_LogmapH.h.invokeExact(m.ptr, Hm.ptr));
        }

        @Override
        public Point2 Expmap(Vector2 v) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_Expmap.h.invokeExact(v.ptr));
        }

        @Override
        public Point2 Expmap(Vector2 v, Matrix Hv) throws Throwable {
            return new Point2((MemorySegment) FF.Point2_ExpmapH.h.invokeExact(v.ptr, Hv.ptr));
        }

        @Override
        public Point2 Retract(Vector2 v) throws Throwable {
            throw new UnsupportedOperationException();
        }

        @Override
        public Vector2 LocalCoordinates(Point2 g) throws Throwable {
            throw new UnsupportedOperationException();
        }

        @Override
        public Point2 Retract(Vector2 v, Matrix H) throws Throwable {
            throw new UnsupportedOperationException();
        }

        @Override
        public Vector2 LocalCoordinates(Point2 g, Matrix H) throws Throwable {
            throw new UnsupportedOperationException();
        }
    }

    public static final Statics statics = new Statics();

    @Override
    public Statics statics() {
        return statics;
    }

    // Point2 seems to hvae the *trait* but not the *method* since it's just "plus"
    // this is really at "VectorSpace" trait.
    // TODO: implement VectorSpace as a type of LieGroup.
    @Override
    public Point2 compose(Point2 h) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 compose(Point2 h, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 between(Point2 h) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 between(Point2 h, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 inverse() throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 inverse(Matrix H) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Matrix AdjointMap() throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 expmap(Vector2 v, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector2 logmap(Point2 g, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 retract(Vector2 v, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector2 localCoordinates(Point2 g, Matrix H1, Matrix H2) throws Throwable {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector2 dxZero() throws Throwable {
        return new Vector2(0, 0);
    }

    public Point2(MemorySegment p) {
        super(p, FF.Point2_delete.h);
    }

    public Point2(double x, double y) throws Throwable {
        this((MemorySegment) FF.Point2.h.invokeExact(x, y));
    }

    public double x() throws Throwable {
        return (double) FF.Point2_x.h.invokeExact(ptr);
    }

    public double y() throws Throwable {
        return (double) FF.Point2_y.h.invokeExact(ptr);
    }

    public Point2 plus(Point2 other) throws Throwable {
        return new Point2((MemorySegment) FF.Point2_plus.h.invokeExact(ptr, other.ptr));
    }

    public Point2 minus(Point2 other) throws Throwable {
        return new Point2((MemorySegment) FF.Point2_minus.h.invokeExact(ptr, other.ptr));
    }

    public Point2 times(double a) throws Throwable {
        return new Point2((MemorySegment) FF.Point2_times.h.invokeExact(ptr, a));
    }

    @Override
    public int dimension() throws Throwable {
        return 2;
    }

    // TODO: use C++ here
    @Override
    public Vector2 localCoordinates(Point2 other) throws Throwable {
        throw new UnsupportedOperationException();
    }

    // TODO: use C++ here
    @Override
    public Point2 retract(Vector2 v) throws Throwable {
        throw new UnsupportedOperationException();
    }

    public static double norm2(Point2 p) throws Throwable {
        return (double) FF.Point2_norm2.h.invokeExact(p.ptr);
    }

    public static double norm2(Point2 p, Matrix H) throws Throwable {
        return (double) FF.Point2_norm2H.h.invokeExact(p.ptr, H.ptr);
    }

    public static double distance2(Point2 p, Point2 q) throws Throwable {
        return (double) FF.Point2_distance2.h.invokeExact(p.ptr, q.ptr);
    }

    public static double distance2(Point2 p, Point2 q, Matrix H1, Matrix H2) throws Throwable {
        return (double) FF.Point2_distance2H.h.invokeExact(p.ptr, q.ptr, H1.ptr, H2.ptr);
    }

    public double norm() throws Throwable {
        return (double) FF.Point2_norm.h.invokeExact(ptr);
    }

    public static boolean check_group_invariants(Point2 a, Point2 b) throws Throwable {
        return (boolean) FF.Point2_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Point2 a, Point2 b) throws Throwable {
        return (boolean) FF.Point2_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public Point2 normalized() throws Throwable {
        return new Point2((MemorySegment) FF.Point2_normalized.h.invokeExact(ptr));
    }

}
