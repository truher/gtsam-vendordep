package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix3 extends ForeignObject {
    public enum FF {
        Matrix3(ADDRESS,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
                JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE),
        Matrix3_delete(null, ADDRESS),
        Matrix3_at(JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT),
        Matrix3_unaryMinus(ADDRESS, ADDRESS),
        Matrix3_identity(ADDRESS),
        Matrix3_compose(ADDRESS, ADDRESS, ADDRESS),
        Matrix3_plus(ADDRESS, ADDRESS, ADDRESS),
        Matrix3_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Matrix3_timesVector3(ADDRESS, ADDRESS, ADDRESS),
        Matrix3_skewSymmetric(ADDRESS,ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Matrix3(MemorySegment p) {
        super(p, FF.Matrix3_delete.h);
    }

    public Matrix3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) FF.Matrix3.h.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public double at(int r, int c) throws Throwable {
        return (double) FF.Matrix3_at.h.invokeExact(ptr, r, c);
    }

    public Matrix3 unaryMinus() throws Throwable {
        return new Matrix3(
                (MemorySegment) FF.Matrix3_unaryMinus.h.invokeExact(ptr));
    }

    public static Matrix3 identity() throws Throwable {
        return new Matrix3((MemorySegment) FF.Matrix3_identity.h.invokeExact());
    }

    public Matrix3 compose(Matrix3 other) throws Throwable {
        return new Matrix3((MemorySegment) FF.Matrix3_compose.h.invokeExact(ptr, other.ptr));
    }

    public Matrix3 plus(Matrix3 other) throws Throwable {
        return new Matrix3((MemorySegment) FF.Matrix3_plus.h.invokeExact(ptr, other.ptr));
    }

    public Matrix3 times(double a) throws Throwable {
        return new Matrix3((MemorySegment) FF.Matrix3_times.h.invokeExact(ptr, a));
    }

    public Vector3 times(Vector3 v) throws Throwable {
        return new Vector3((MemorySegment) FF.Matrix3_timesVector3.h.invokeExact(ptr, v.ptr));
    }

    public static Matrix3 skewSymmetric(Vector3 v) throws Throwable {
        return new Matrix3((MemorySegment) FF.Matrix3_skewSymmetric.h.invokeExact(v.ptr));
    }
}
