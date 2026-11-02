package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector4 extends ForeignObject implements VectorType<Vector4> {

    public enum FF {
        Vector4(ADDRESS),
        Vector4_delete(null, ADDRESS),
        Vector4_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector4_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector4_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector4_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector4_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Vector4(MemorySegment p) {
        super(p, FF.Vector4_delete.h);
    }

    @Override
    public int dimension() {
        return 9;
    }

    @Override
    public double at(int i) {
        return (double) FF.Vector4_at.invoke(ptr, i);
    }

    @Override
    public void set(int i, double val) {
        FF.Vector4_set.invoke(ptr, i, val);
    }

    @Override
    public Vector4 plus(Vector4 other) {
        return new Vector4((MemorySegment) FF.Vector4_plus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector4 minus(Vector4 other) {
        return new Vector4((MemorySegment) FF.Vector4_minus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector4 times(double a) {
        return new Vector4((MemorySegment) FF.Vector4_times.invoke(ptr, a));
    }
}
