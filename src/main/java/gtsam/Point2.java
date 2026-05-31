package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Point2 extends ForeignObject implements Manifold<Point2, Vector2> {
    public enum FF {
        Point2(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE),
        Point2_delete(null, ADDRESS),
        Point2_x(JAVA_DOUBLE, ADDRESS),
        Point2_y(JAVA_DOUBLE, ADDRESS),
        Point2_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point2_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Traits implements Manifold.Traits<Point2, Vector2> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
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

    @Override
    public int dimension() throws Throwable {
        return 2;
    }

    // TODO: use C++ here
    @Override
    public Vector2 localCoordinates(Point2 other) throws Throwable {
        return new Vector2(other.x() - x(), other.y() - y());
    }

    // TODO: use C++ here
    @Override
    public Point2 retract(Vector2 v) throws Throwable {
        return new Point2(x() + v.at(0), y() + v.at(1));
    }

    public static boolean check_group_invariants(Point2 a, Point2 b) throws Throwable {
        return (boolean) FF.Point2_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Point2 a, Point2 b) throws Throwable {
        return (boolean) FF.Point2_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

}
