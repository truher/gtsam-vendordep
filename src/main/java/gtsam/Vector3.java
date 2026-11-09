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

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Vector3 dxZero() {
        return new Vector3(0, 0, 0);
    }

    public Vector3(MemorySegment p) {
        super(p, FF.Vector3_delete.h);
    }

    /** Initializes to zero. */
    public Vector3() {
        this((MemorySegment) FF.Vector3.invoke());
    }

    public Vector3(double v0, double v1, double v2) {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
    }

    public Vector3(Point3 p) {
        this(p.x(), p.y(), p.z());
    }

    public Vector3(Vector v) {
        this(v.at(0), v.at(1), v.at(2));
    }

    @Override
    public int dimension() {
        return 3;
    }

    @Override
    public double at(int i) {
        return (double) FF.Vector3_at.invoke(ptr, i);
    }

    @Override
    public void set(int i, double val) {
        FF.Vector3_set.invoke(ptr, i, val);
    }

    @Override
    public Vector3 plus(Vector3 other) {
        return new Vector3((MemorySegment) FF.Vector3_plus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector3 minus(Vector3 other) {
        return new Vector3((MemorySegment) FF.Vector3_minus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector3 times(double a) {
        return new Vector3((MemorySegment) FF.Vector3_times.invoke(ptr, a));
    }

    @Override
    public String toString() {
        StringBuilder b = new StringBuilder();
        int rows = 3;
        b.append("[");
        for (int r = 0; r < rows; ++r) {
            b.append(String.format(" %12.8f ", at(r)));
        }
        b.append("]\n");
        return b.toString();
    }

    /** In a vector space, local() is just minus(). */
    @Override
    public Vector3 local(Vector3 other) {
        return other.minus(this);
    }

    /** In a vector space, retract() is just plus(). */
    @Override
    public Vector3 retract(Vector3 v) {
        return plus(v);
    }

    public double norm() {
        return (double) FF.Vector3_norm.invoke(ptr);
    }

    public double dot(Vector3 other) {
        return (double) FF.Vector3_dot.invoke(ptr, other.ptr);
    }

    public Vector3 normalized() {
        return new Vector3((MemorySegment) FF.Vector3_normalized.invoke(ptr));
    }

    public Vector3 cross(Vector3 other) {
        return new Vector3((MemorySegment) FF.Vector3_cross.invoke(ptr, other.ptr));
    }

    @Override
    public Vector3 local(Vector3 g, Matrix H1, Matrix H2) {
        Vector3 result = local(g);
        H1.set(new double[][] { { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 } });
        H2.set(new double[][] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
        return result;
    }

    @Override
    public Vector3 retract(Vector3 v, Matrix H1, Matrix H2) {
        Vector3 result = retract(v);
        H1.set(new double[][] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
        H2.set(new double[][] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });
        return result;
    }
}
