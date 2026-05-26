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
public class Vector9 extends ForeignObject implements VectorType<Vector9> {
    public enum FF {
        Vector9(ADDRESS),
        Vector9_delete(null, ADDRESS),
        Vector9_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector9_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector9_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector9_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector9_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Vector9(MemorySegment p) {
        super(p, FF.Vector9_delete.h);
    }

    public Vector9() throws Throwable {
        this((MemorySegment) FF.Vector9.h.invokeExact());
    }

    public Vector9(//
            double v0, double v1, double v2, //
            double v3, double v4, double v5, //
            double v6, double v7, double v8) throws Throwable {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
        set(3, v3);
        set(4, v4);
        set(5, v5);
        set(6, v6);
        set(7, v7);
        set(8, v8);
    }

    @Override
    public int dimension() throws Throwable {
        return 9;
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) FF.Vector9_at.h.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        FF.Vector9_set.h.invokeExact(ptr, i, val);
    }

    @Override
    public Vector9 plus(Vector9 other) throws Throwable {
        return new Vector9((MemorySegment) FF.Vector9_plus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector9 minus(Vector9 other) throws Throwable {
        return new Vector9((MemorySegment) FF.Vector9_minus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector9 times(double a) throws Throwable {
        return new Vector9((MemorySegment) FF.Vector9_times.h.invokeExact(ptr, a));
    }

}
