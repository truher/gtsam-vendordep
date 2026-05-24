package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix2 extends ForeignObject {
    private static final MethodHandle Matrix2 = Lib.down(
            "Matrix2", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Matrix2_delete = Lib.downVoid(
            "Matrix2_delete", ADDRESS);
    private static final MethodHandle Matrix2_at = Lib.down(
            "Matrix2_at", JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix2_unaryMinus = Lib.down(
            "Matrix2_unaryMinus", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix2_identity = Lib.down(
            "Matrix2_identity", ADDRESS);
    private static final MethodHandle Matrix2_compose = Lib.down(
            "Matrix2_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix2_plus = Lib.down(
            "Matrix2_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix2_times = Lib.down(
            "Matrix2_times", ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Matrix2(MemorySegment p) {
        super(p, Matrix2_delete);
    }

    public Matrix2( //
            double R11, double R12, //
            double R21, double R22) throws Throwable {
        this((MemorySegment) Matrix2.invokeExact(//
                R11, R12, //
                R21, R22));
    }

    public double at(int r, int c) throws Throwable {
        return (double) Matrix2_at.invokeExact(ptr, r, c);
    }

    public Matrix2 unaryMinus() throws Throwable {
        return new Matrix2(
                (MemorySegment) Matrix2_unaryMinus.invokeExact(ptr));
    }

    public static Matrix2 identity() throws Throwable {
        return new Matrix2((MemorySegment) Matrix2_identity.invokeExact());
    }

    public Matrix2 compose(Matrix2 other) throws Throwable {
        return new Matrix2((MemorySegment) Matrix2_compose.invokeExact(ptr, other.ptr));
    }

    public Matrix2 plus(Matrix2 other) throws Throwable {
        return new Matrix2((MemorySegment) Matrix2_plus.invokeExact(ptr, other.ptr));
    }

    public Matrix2 times(double a) throws Throwable {
        return new Matrix2((MemorySegment) Matrix2_times.invokeExact(ptr, a));
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = 2;
            int cols = 2;
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
