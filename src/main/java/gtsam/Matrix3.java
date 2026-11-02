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
        Matrix3_col(ADDRESS, ADDRESS, JAVA_INT),
        Matrix3_unaryMinus(ADDRESS, ADDRESS),
        Matrix3_identity(ADDRESS),
        Matrix3_compose(ADDRESS, ADDRESS, ADDRESS),
        Matrix3_plus(ADDRESS, ADDRESS, ADDRESS),
        Matrix3_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Matrix3_timesVector3(ADDRESS, ADDRESS, ADDRESS),
        Matrix3_skewSymmetric(ADDRESS, ADDRESS),
        Matrix3_determinant(JAVA_DOUBLE, ADDRESS),
        Matrix3_transpose(ADDRESS, ADDRESS),
        Matrix3_norm(JAVA_DOUBLE, ADDRESS),
        Matrix3_inverse(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Matrix3(MemorySegment p) {
        super(p, FF.Matrix3_delete.h);
    }

    public Matrix3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) {
        this((MemorySegment) FF.Matrix3.invoke(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public double at(int r, int c) {
        return (double) FF.Matrix3_at.invoke(ptr, r, c);
    }

    public Vector3 col(int c) {
        return new Vector3((MemorySegment) FF.Matrix3_col.invoke(ptr, c));
    }

    public Matrix3 unaryMinus() {
        return new Matrix3(
                (MemorySegment) FF.Matrix3_unaryMinus.invoke(ptr));
    }

    public static Matrix3 identity() {
        return new Matrix3((MemorySegment) FF.Matrix3_identity.invoke());
    }

    public Matrix3 compose(Matrix3 other) {
        return new Matrix3((MemorySegment) FF.Matrix3_compose.invoke(ptr, other.ptr));
    }

    public Matrix3 plus(Matrix3 other) {
        return new Matrix3((MemorySegment) FF.Matrix3_plus.invoke(ptr, other.ptr));
    }

    public Matrix3 minus(Matrix3 other) {
        return plus(other.times(-1));
    }

    public Matrix3 times(double a) {
        return new Matrix3((MemorySegment) FF.Matrix3_times.invoke(ptr, a));
    }

    public Vector3 times(Vector3 v) {
        return new Vector3((MemorySegment) FF.Matrix3_timesVector3.invoke(ptr, v.ptr));
    }

    public static Matrix3 skewSymmetric(Vector3 v) {
        return new Matrix3((MemorySegment) FF.Matrix3_skewSymmetric.invoke(v.ptr));
    }

    public double determinant() {
        return (double) FF.Matrix3_determinant.invoke(ptr);
    }

    public Matrix3 transpose() {
        return new Matrix3((MemorySegment) FF.Matrix3_transpose.invoke(ptr));
    }

    public double norm() {
        return (double) FF.Matrix3_norm.invoke(ptr);
    }

    public Matrix3 inverse() {
        return new Matrix3((MemorySegment) FF.Matrix3_inverse.invoke(ptr));
    }
}
