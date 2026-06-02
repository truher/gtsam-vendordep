package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

// TODO: finish implementation
public class Vector6 extends ForeignObject
        implements Manifold<Vector6, Vector6>, VectorType<Vector6> {
    public enum FF {
        Vector6(ADDRESS),
        Vector6_delete(null, ADDRESS),
        Vector6_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector6_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector6_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector6_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector6_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Vector6(MemorySegment p) {
        super(p, FF.Vector6_delete.h);
    }

    public Vector6() throws Throwable {
        this((MemorySegment) FF.Vector6.h.invokeExact());
    }

    public Vector6(//
            double v0, double v1, double v2, //
            double v3, double v4, double v5) throws Throwable {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
        set(3, v3);
        set(4, v4);
        set(5, v5);
    }

    @Override
    public int dimension() throws Throwable {
        return 6;
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) FF.Vector6_at.h.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        FF.Vector6_set.h.invokeExact(ptr, i, val);
    }

    @Override
    public Vector6 plus(Vector6 other) throws Throwable {
        return new Vector6((MemorySegment) FF.Vector6_plus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector6 minus(Vector6 other) throws Throwable {
        return new Vector6((MemorySegment) FF.Vector6_minus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector6 times(double a) throws Throwable {
        return new Vector6((MemorySegment) FF.Vector6_times.h.invokeExact(ptr, a));
    }

    public static class Traits implements Manifold.Traits<Vector6, Vector6> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector6 dxZero() throws Throwable {
        return new Vector6(0, 0, 0, 0, 0, 0);
    }

    @Override
    public Vector6 localCoordinates(Vector6 other) throws Throwable {
        return other.minus(this);
    }

    @Override
    public Vector6 retract(Vector6 v) throws Throwable {
        return plus(v);
    }

}
