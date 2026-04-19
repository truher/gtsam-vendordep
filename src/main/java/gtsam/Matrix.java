package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix extends ForeignObject {
    private static final MethodHandle Matrix = Lib.down(
            "Matrix", ADDRESS);
    private static final MethodHandle Matrix_delete = Lib.downVoid(
            "Matrix_delete", ADDRESS);
    private static final MethodHandle Matrix_Matrix3 = Lib.down(
            "Matrix_Matrix3", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_at = Lib.down(
            "Matrix_at", JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix_diagonal_cwiseSqrt = Lib.down(
            "Matrix_diagonal_cwiseSqrt", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_rows = Lib.down(
            "Matrix_rows", JAVA_INT, ADDRESS);
    private static final MethodHandle Matrix_cols = Lib.down(
            "Matrix_cols", JAVA_INT, ADDRESS);

    public Matrix(MemorySegment p) {
        super(p, Matrix_delete);
    }

    public Matrix() throws Throwable {
        this((MemorySegment) Matrix.invokeExact());
    }

    public Matrix(Matrix3 m) throws Throwable {
        this((MemorySegment) Matrix_Matrix3.invokeExact(m.ptr));
    }

    public double at(int r, int c) throws Throwable {
        return (double) Matrix_at.invokeExact(ptr, r, c);
    }

    public Vector diagonal_cwiseSqrt() throws Throwable {
        return new Vector(
                (MemorySegment) Matrix_diagonal_cwiseSqrt.invokeExact(ptr));
    }

    public int rows() throws Throwable {
        return (int) Matrix_rows.invokeExact(ptr);
    }

    public int cols() throws Throwable {
        return (int) Matrix_cols.invokeExact(ptr);
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = rows();
            int cols = cols();
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
