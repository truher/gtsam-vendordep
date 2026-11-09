package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector1 extends ForeignObject
        implements VectorType<Vector1>, Manifold<Vector1, Vector1> {
    public enum FF {
        Vector1(ADDRESS, JAVA_DOUBLE),
        Vector1_delete(null, ADDRESS),
        Vector1_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector1_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector1_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector1_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector1_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Vector1 dxZero() {
        return new Vector1(0);
    }

    public Vector1(MemorySegment p) {
        super(p, FF.Vector1_delete.h);
    }

    public Vector1(double v0) {
        this((MemorySegment) FF.Vector1.invoke(v0));
    }

    @Override
    public int dimension() {
        return 1;
    }

    @Override
    public double at(int i) {
        return (double) FF.Vector1_at.invoke(ptr, i);
    }

    @Override
    public void set(int i, double val) {
        FF.Vector1_set.invoke(ptr, i, val);
    }

    @Override
    public Vector1 plus(Vector1 other) {
        return new Vector1((MemorySegment) FF.Vector1_plus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector1 minus(Vector1 other) {
        return new Vector1((MemorySegment) FF.Vector1_minus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector1 times(double a) {
        return new Vector1((MemorySegment) FF.Vector1_times.invoke(ptr, a));
    }

    /** In a vector space, local() is just minus(). */
    @Override
    public Vector1 local(Vector1 other) {
        return other.minus(this);
    }

    /** In a vector space, retract() is just plus(). */
    @Override
    public Vector1 retract(Vector1 v) {
        return plus(v);
    }

    @Override
    public Vector1 local(Vector1 g, Matrix H1, Matrix H2) {
        Vector1 result = local(g);
        H1.set(new double[][] { { -1 } });
        H2.set(new double[][] { { 1 } });
        return result;
    }

    @Override
    public Vector1 retract(Vector1 v, Matrix H1, Matrix H2) {
        Vector1 result = retract(v);
        H1.set(new double[][] { { 1 } });
        H2.set(new double[][] { { 1 } });
        return result;
    }
}
