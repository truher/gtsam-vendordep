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
        implements VectorType<Vector1>, Manifold<Vector1, Vector1>  {
    public enum FF {
        Vector1(ADDRESS, JAVA_DOUBLE),
        Vector1_delete(null, ADDRESS),
        Vector1_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector1_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector1_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector1_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector1_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

        public static class Traits implements Manifold.Traits<Vector1, Vector1> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector1 dxZero() throws Throwable {
        return new Vector1(0);
    }

    public Vector1(MemorySegment p) {
        super(p, FF.Vector1_delete.h);
    }

    public Vector1(double v0) throws Throwable {
        this((MemorySegment) FF.Vector1.h.invokeExact(v0));
    }

    @Override
    public int dimension() throws Throwable {
        return 1;
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) FF.Vector1_at.h.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        FF.Vector1_set.h.invokeExact(ptr, i, val);
    }

    @Override
    public Vector1 plus(Vector1 other) throws Throwable {
        return new Vector1((MemorySegment) FF.Vector1_plus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector1 minus(Vector1 other) throws Throwable {
        return new Vector1((MemorySegment) FF.Vector1_minus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector1 times(double a) throws Throwable {
        return new Vector1((MemorySegment) FF.Vector1_times.h.invokeExact(ptr, a));
    }

        @Override
    public Vector1 localCoordinates(Vector1 other) throws Throwable {
        return other.minus(this);
    }

    @Override
    public Vector1 retract(Vector1 v) throws Throwable {
        return plus(v);
    }
}
