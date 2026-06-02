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
 */
public class Point3 extends ForeignObject
        implements Manifold<Point3, Vector3> {
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
        Point3_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point3_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS);

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

    public static class Traits implements Manifold.Traits<Point3, Vector3> {

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

    @Override
    public Vector3 localCoordinates(Point3 other) throws Throwable {
        return new Vector3(other.minus(this));
    }

    @Override
    public Point3 retract(Vector3 v) throws Throwable {
        return plus(new Point3(v));
    }

    public Point3 cross(Point3 q) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_cross.h.invokeExact(ptr, q.ptr));
    }

    public static Point3 cross(Point3 p, Point3 q) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_crossPoint3Point3.h.invokeExact(p.ptr, q.ptr));
    }

}
