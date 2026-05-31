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
public class Point3 extends ForeignObject {
    public enum FF {
        Point3(ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Point3_delete(null, ADDRESS),
        Point3_x(JAVA_DOUBLE, ADDRESS),
        Point3_y(JAVA_DOUBLE, ADDRESS),
        Point3_z(JAVA_DOUBLE, ADDRESS),
        Point3_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Point3_check_group_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        Point3_check_manifold_invariants(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Point3(MemorySegment p) {
        super(p, FF.Point3_delete.h);
    }

    public Point3(double x, double y, double z) throws Throwable {
        this((MemorySegment) FF.Point3.h.invokeExact(x, y, z));
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

    public Point3 times(double a) throws Throwable {
        return new Point3((MemorySegment) FF.Point3_times.h.invokeExact(ptr, a));
    }

    public static boolean check_group_invariants(Point3 a, Point3 b) throws Throwable {
        return (boolean) FF.Point3_check_group_invariants.h.invokeExact(a.ptr, b.ptr);
    }

    public static boolean check_manifold_invariants(Point3 a, Point3 b) throws Throwable {
        return (boolean) FF.Point3_check_manifold_invariants.h.invokeExact(a.ptr, b.ptr);
    }

}
