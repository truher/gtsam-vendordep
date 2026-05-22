package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;


import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix3 extends ForeignObject {
    private static final MethodHandle Matrix3 = Lib.down(
            "Matrix3", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Matrix3_delete = Lib.downVoid(
            "Matrix3_delete", ADDRESS);
    private static final MethodHandle Matrix3_at = Lib.down(
            "Matrix3_at", JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix3_unaryMinus = Lib.down(
            "Matrix3_unaryMinus", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix3_identity = Lib.down(
            "Matrix3_identity", ADDRESS);
    private static final MethodHandle Matrix3_equals = Lib.down(
            "Matrix3_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Matrix3_compose = Lib.down(
            "Matrix3_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix3_plus = Lib.down(
            "Matrix3_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix3_times = Lib.down(
            "Matrix3_times", ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Matrix3(MemorySegment p) {
        super(p, Matrix3_delete);
    }

    public Matrix3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) Matrix3.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public double at(int r, int c) throws Throwable {
        return (double) Matrix3_at.invokeExact(ptr, r, c);
    }

    public Matrix3 unaryMinus() throws Throwable {
        return new Matrix3(
                (MemorySegment) Matrix3_unaryMinus.invokeExact(ptr));
    }

    public static Matrix3 identity() throws Throwable {
        return new Matrix3((MemorySegment) Matrix3_identity.invokeExact());
    }

    public boolean equals(Matrix3 other, double tol) throws Throwable {
        return (boolean) Matrix3_equals.invokeExact(ptr, other.ptr, tol);
    }

    public Matrix3 compose(Matrix3 other) throws Throwable {
        return new Matrix3((MemorySegment) Matrix3_compose.invokeExact(ptr, other.ptr));
    }

    public Matrix3 plus(Matrix3 other) throws Throwable {
        return new Matrix3((MemorySegment) Matrix3_plus.invokeExact(ptr, other.ptr));
    }

    public Matrix3 times(double a) throws Throwable {
        return new Matrix3((MemorySegment) Matrix3_times.invokeExact(ptr, a));
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = 3;
            int cols = 3;
            b.append("[\n");
            for (int r = 0; r < rows; ++r) {
                b.append("  [");
                for (int c = 0; c < cols; ++c) {
                    b.append(String.format(" %6.3f ", at(r, c)));
                }
                b.append("]\n");
            }
            b.append("]\n");
            return b.toString();
        } catch (Throwable e) {
            e.printStackTrace();
            return "";
        }
    }

}
