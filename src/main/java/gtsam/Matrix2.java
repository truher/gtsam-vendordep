package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix2 extends ForeignObject {
    public enum FF {
        Matrix2(ADDRESS,
                JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE),
        Matrix2_delete(null, ADDRESS),
        Matrix2_at(JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT),
        Matrix2_unaryMinus(ADDRESS, ADDRESS),
        Matrix2_identity(ADDRESS),
        Matrix2_compose(ADDRESS, ADDRESS, ADDRESS),
        Matrix2_plus(ADDRESS, ADDRESS, ADDRESS),
        Matrix2_times(ADDRESS, ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Matrix2(MemorySegment p) {
        super(p, FF.Matrix2_delete.h);
    }

    public Matrix2( //
            double R11, double R12, //
            double R21, double R22) {
        this((MemorySegment) FF.Matrix2.invoke(//
                R11, R12, //
                R21, R22));
    }

    public double at(int r, int c) {
        return (double) FF.Matrix2_at.invoke(ptr, r, c);
    }

    public Matrix2 unaryMinus() {
        return new Matrix2(
                (MemorySegment) FF.Matrix2_unaryMinus.invoke(ptr));
    }

    public static Matrix2 identity() {
        return new Matrix2((MemorySegment) FF.Matrix2_identity.invoke());
    }

    public Matrix2 compose(Matrix2 other) {
        return new Matrix2((MemorySegment) FF.Matrix2_compose.invoke(ptr, other.ptr));
    }

    public Matrix2 plus(Matrix2 other) {
        return new Matrix2((MemorySegment) FF.Matrix2_plus.invoke(ptr, other.ptr));
    }

    public Matrix2 times(double a) {
        return new Matrix2((MemorySegment) FF.Matrix2_times.invoke(ptr, a));
    }
}
