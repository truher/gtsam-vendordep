package gtsam;

package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix6 extends ForeignObject {
    private static final MethodHandle Matrix6 = Lib.down(
            "Matrix6", ADDRESS);
    private static final MethodHandle Matrix6_delete = Lib.downVoid(
            "Matrix6_delete", ADDRESS);
    private static final MethodHandle Matrix6_at = Lib.down(
            "Matrix6_at", JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix6_unaryMinus = Lib.down(
            "Matrix6_unaryMinus", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix6_identity = Lib.down(
            "Matrix6_identity", ADDRESS);
    private static final MethodHandle Matrix6_equals = Lib.down(
            "Matrix6_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Matrix6_compose = Lib.down(
            "Matrix6_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix6_plus = Lib.down(
            "Matrix6_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix6_times = Lib.down(
            "Matrix6_times", ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Matrix6_inverse = Lib.down(
            "Matrix6_inverse", ADDRESS, ADDRESS);

    public Matrix6(MemorySegment p) {
        super(p, Matrix6_delete);
    }

    public Matrix6( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) Matrix6.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

    public double at(int r, int c) throws Throwable {
        return (double) Matrix6_at.invokeExact(ptr, r, c);
    }

    public Matrix6 unaryMinus() throws Throwable {
        return new Matrix6(
                (MemorySegment) Matrix6_unaryMinus.invokeExact(ptr));
    }

    public static Matrix6 identity() throws Throwable {
        return new Matrix6((MemorySegment) Matrix6_identity.invokeExact());
    }

    public boolean equals(Matrix6 other, double tol) throws Throwable {
        return (boolean) Matrix6_equals.invokeExact(ptr, other.ptr, tol);
    }

    public Matrix6 compose(Matrix6 other) throws Throwable {
        return new Matrix6((MemorySegment) Matrix6_compose.invokeExact(ptr, other.ptr));
    }

    public Matrix6 plus(Matrix6 other) throws Throwable {
        return new Matrix6((MemorySegment) Matrix6_plus.invokeExact(ptr, other.ptr));
    }

    public Matrix6 times(double a) throws Throwable {
        return new Matrix6((MemorySegment) Matrix6_times.invokeExact(ptr, a));
    }

    public Matrix6 inverse() throws Throwable {
        System.out.println("Matrix6.inverse()");
        return new Matrix6((MemorySegment) Matrix6_inverse.invokeExact(ptr));
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = 6;
            int cols = 6;
            b.append("[\n");
            for (int r = 0; r < rows; ++r) {
                b.append("  [");
                for (int c = 0; c < cols; ++c) {
                    b.append(String.format(" %9.6f ", at(r, c)));
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
