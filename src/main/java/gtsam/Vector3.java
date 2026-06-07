package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector3 extends ForeignObject
        implements VectorType<Vector3>, Manifold<Vector3, Vector3> {
    public enum FF {
        Vector3(ADDRESS),
        Vector3_delete(null, ADDRESS),
        Vector3_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector3_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector3_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector3_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector3_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Vector3_norm(JAVA_DOUBLE, ADDRESS),
        Vector3_dot(JAVA_DOUBLE, ADDRESS, ADDRESS),
        Vector3_normalized(ADDRESS, ADDRESS),
        Vector3_cross(ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Companion implements Manifold.Companion<Vector3, Vector3> {

    }

    public static final Companion companion = new Companion();

    @Override
    public Companion companion() {
        return companion;
    }

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3(0, 0, 0);
    }

    public Vector3(MemorySegment p) {
        super(p, FF.Vector3_delete.h);
    }

    /** Initializes to zero. */
    public Vector3() throws Throwable {
        this((MemorySegment) FF.Vector3.h.invokeExact());
    }

    public Vector3(double v0, double v1, double v2) throws Throwable {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
    }

    public Vector3(Point3 p) throws Throwable {
        this(p.x(), p.y(), p.z());
    }

    public Vector3(Vector v) throws Throwable {
        this(v.at(0), v.at(1), v.at(2));
    }

    @Override
    public int dimension() throws Throwable {
        return 3;
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) FF.Vector3_at.h.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        FF.Vector3_set.h.invokeExact(ptr, i, val);
    }

    @Override
    public Vector3 plus(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) FF.Vector3_plus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector3 minus(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) FF.Vector3_minus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector3 times(double a) throws Throwable {
        return new Vector3((MemorySegment) FF.Vector3_times.h.invokeExact(ptr, a));
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = 3;
            b.append("[");
            for (int r = 0; r < rows; ++r) {
                b.append(String.format(" %8.5f ", at(r)));
            }
            b.append("]\n");
            return b.toString();
        } catch (Throwable e) {
            e.printStackTrace();
            return "";
        }
    }

    @Override
    public Vector3 localCoordinates(Vector3 other) throws Throwable {
        return other.minus(this);
    }

    @Override
    public Vector3 retract(Vector3 v) throws Throwable {
        return plus(v);
    }

    public double norm() throws Throwable {
        return (double) FF.Vector3_norm.h.invokeExact(ptr);
    }

    public double dot(Vector3 other) throws Throwable {
        return (double) FF.Vector3_dot.h.invokeExact(ptr, other.ptr);
    }

    public Vector3 normalized() throws Throwable {
        return new Vector3((MemorySegment) FF.Vector3_normalized.h.invokeExact(ptr));
    }

    public Vector3 cross(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) FF.Vector3_cross.h.invokeExact(ptr, other.ptr));
    }
}
