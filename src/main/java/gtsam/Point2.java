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
        Point2_Local(ADDRESS, ADDRESS, ADDRESS),
        Point2_Retract(ADDRESS, ADDRESS, ADDRESS),
        Point2_logmap(ADDRESS, ADDRESS, ADDRESS),
        Point2_expmap(ADDRESS, ADDRESS, ADDRESS),
        Point2_Compose(ADDRESS, ADDRESS, ADDRESS),
        Point2_ComposeH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point2_Between(ADDRESS, ADDRESS, ADDRESS),
        Point2_BetweenH(ADDRESS, ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        Point2_Inverse(ADDRESS, ADDRESS),
        Point2_InverseH(ADDRESS, ADDRESS, ADDRESS),
        Point2_AdjointMap(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Point2 compose(Point2 h) {
        return new Point2((MemorySegment) FF.Point2_Compose.invoke(ptr, h.ptr));
    }

    @Override
    public Point2 compose(Point2 h, Matrix H1, Matrix H2) {
        return new Point2((MemorySegment) FF.Point2_ComposeH.invoke(
                ptr, h.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Point2 between(Point2 h) {
        return new Point2((MemorySegment) FF.Point2_Between.invoke(ptr, h.ptr));
    }

    @Override
    public Point2 between(Point2 h, Matrix H1, Matrix H2) {
        return new Point2((MemorySegment) FF.Point2_BetweenH.invoke(
                ptr, h.ptr, H1.ptr, H2.ptr));
    }

    @Override
    public Point2 inverse() {
        return new Point2((MemorySegment) FF.Point2_Inverse.invoke(ptr));
    }

    @Override
    public Point2 inverse(Matrix H) {
        return new Point2((MemorySegment) FF.Point2_InverseH.invoke(
                ptr, H.ptr));
    }

    @Override
    public Matrix AdjointMap() {
        return new Matrix((MemorySegment) FF.Point2_AdjointMap.invoke(ptr));
    }

    @Override
    public Point2 expmap(Vector2 v) {
        return new Point2((MemorySegment) FF.Point2_expmap.invoke(ptr, v.ptr));
    }

    @Override
    public Point2 expmap(Vector2 v, Matrix H1, Matrix H2) {
        throw new UnsupportedOperationException();
    }

    public Vector2 logmap(Point2 g) {
        return new Vector2((MemorySegment) FF.Point2_logmap.invoke(ptr, g.ptr));
    }

    @Override
    public Vector2 logmap(Point2 g, Matrix H1, Matrix H2) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Point2 retract(Vector2 v, Matrix H1, Matrix H2) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector2 local(Point2 g, Matrix H1, Matrix H2) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector2 dxZero() {
        return new Vector2(0, 0);
    }

    public Point2(MemorySegment p) {
        super(p, FF.Point2_delete.h);
    }

    public Point2() {
        this(0, 0);
    }

    public Point2(double x, double y) {
        this((MemorySegment) FF.Point2.invoke(x, y));
    }

    public double x() {
        return (double) FF.Point2_x.invoke(ptr);
    }

    public double y() {
        return (double) FF.Point2_y.invoke(ptr);
    }

    public Point2 plus(Point2 other) {
        return new Point2((MemorySegment) FF.Point2_plus.invoke(ptr, other.ptr));
    }

    public Point2 minus(Point2 other) {
        return new Point2((MemorySegment) FF.Point2_minus.invoke(ptr, other.ptr));
    }

    public Point2 times(double a) {
        return new Point2((MemorySegment) FF.Point2_times.invoke(ptr, a));
    }

    @Override
    public int dimension() {
        return 2;
    }

    @Override
    public Vector2 local(Point2 other) {
        return new Vector2((MemorySegment) FF.Point2_Local.invoke(ptr, other.ptr));
    }

    @Override
    public Point2 retract(Vector2 v) {
        return new Point2((MemorySegment) FF.Point2_Retract.invoke(ptr, v.ptr));
    }

    public static double norm2(Point2 p) {
        return (double) FF.Point2_norm2.invoke(p.ptr);
    }

    public static double norm2(Point2 p, Matrix H) {
        return (double) FF.Point2_norm2H.invoke(p.ptr, H.ptr);
    }

    public static double distance2(Point2 p, Point2 q) {
        return (double) FF.Point2_distance2.invoke(p.ptr, q.ptr);
    }

    public static double distance2(Point2 p, Point2 q, Matrix H1, Matrix H2) {
        return (double) FF.Point2_distance2H.invoke(p.ptr, q.ptr, H1.ptr, H2.ptr);
    }

    public double norm() {
        return (double) FF.Point2_norm.invoke(ptr);
    }

    public static boolean check_group_invariants(Point2 a, Point2 b) {
        return (boolean) FF.Point2_check_group_invariants.invoke(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Point2 a, Point2 b) {
        return (boolean) FF.Point2_check_manifold_invariants.invoke(a.ptr, b.ptr);
    }

    public Point2 normalized() {
        return new Point2((MemorySegment) FF.Point2_normalized.invoke(ptr));
    }

}
