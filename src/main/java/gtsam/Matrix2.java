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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Matrix2(MemorySegment p) {
        super(p, FF.Matrix2_delete.h);
    }

    public Matrix2( //
            double R11, double R12, //
            double R21, double R22) throws Throwable {
        this((MemorySegment) FF.Matrix2.h.invokeExact(//
                R11, R12, //
                R21, R22));
    }

    public double at(int r, int c) throws Throwable {
        return (double) FF.Matrix2_at.h.invokeExact(ptr, r, c);
    }

    public Matrix2 unaryMinus() throws Throwable {
        return new Matrix2(
                (MemorySegment) FF.Matrix2_unaryMinus.h.invokeExact(ptr));
    }

    public static Matrix2 identity() throws Throwable {
        return new Matrix2((MemorySegment) FF.Matrix2_identity.h.invokeExact());
    }

    public Matrix2 compose(Matrix2 other) throws Throwable {
        return new Matrix2((MemorySegment) FF.Matrix2_compose.h.invokeExact(ptr, other.ptr));
    }

    public Matrix2 plus(Matrix2 other) throws Throwable {
        return new Matrix2((MemorySegment) FF.Matrix2_plus.h.invokeExact(ptr, other.ptr));
    }

    public Matrix2 times(double a) throws Throwable {
        return new Matrix2((MemorySegment) FF.Matrix2_times.h.invokeExact(ptr, a));
    }
}
