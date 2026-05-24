package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * Dynamically-dimensioned Matrix, corresponds to Eigen::MatrixXd.
 */
public class Matrix extends ForeignObject {
    private static final MethodHandle Matrix = Lib.down(
            "Matrix", ADDRESS);
    private static final MethodHandle Matrix_identity3 = Lib.down(
            "Matrix_identity3", ADDRESS);
    private static final MethodHandle Matrix_withRowsCols = Lib.down(
            "Matrix_withRowsCols", ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix_delete = Lib.downVoid(
            "Matrix_delete", ADDRESS);
    private static final MethodHandle Matrix_Matrix3 = Lib.down(
            "Matrix_Matrix3", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_set = Lib.downVoid(
            "Matrix_set", ADDRESS, JAVA_INT, JAVA_INT, JAVA_DOUBLE);
    // private static final MethodHandle Matrix_setCol = Lib.downVoid(
    // "Matrix_setCol", ADDRESS, JAVA_INT, ADDRESS);
    private static final MethodHandle Matrix_at = Lib.down(
            "Matrix_at", JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix_diagonal_cwiseSqrt = Lib.down(
            "Matrix_diagonal_cwiseSqrt", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_rows = Lib.down(
            "Matrix_rows", JAVA_INT, ADDRESS);
    private static final MethodHandle Matrix_cols = Lib.down(
            "Matrix_cols", JAVA_INT, ADDRESS);
    private static final MethodHandle Matrix_inverse = Lib.down(
            "Matrix_inverse", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_compose = Lib.down(
            "Matrix_compose", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_transpose = Lib.down(
            "Matrix_transpose", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_timesVector3 = Lib.down(
            "Matrix_timesVector3", ADDRESS, ADDRESS, ADDRESS);

    public Matrix(MemorySegment p) {
        super(p, Matrix_delete);
    }

    public Matrix() throws Throwable {
        this((MemorySegment) Matrix.invokeExact());
    }

    public static Matrix identity3() throws Throwable {
        return new Matrix((MemorySegment) Matrix_identity3.invokeExact());
    }

    public Matrix(int rows, int cols) throws Throwable {
        this((MemorySegment) Matrix_withRowsCols.invokeExact(rows, cols));
    }

    public Matrix(Matrix3 m) throws Throwable {
        this((MemorySegment) Matrix_Matrix3.invokeExact(m.ptr));
    }

    public Matrix(double[][] x) throws Throwable {
        int rows = x.length;
        int cols = x[0].length;
        this(rows, cols);
        for (int row = 0; row < rows(); ++row) {
            for (int col = 0; col < cols(); ++col) {
                set(row, col, x[row][col]);
            }
        }
    }

    public void set(int row, int col, double v) throws Throwable {
        Matrix_set.invokeExact(ptr, row, col, v);
    }

    public <V extends VectorType<V>> void setCol(int col, V v) throws Throwable {
        for (int row = 0; row < v.dimension(); ++row) {
            set(row, col, v.at(row));
        }
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

    /** TODO: what if not square? Pseudo-inverse? */
    public Matrix inverse() throws Throwable {
        return new Matrix((MemorySegment) Matrix_inverse.invokeExact(ptr));
    }

    /**
     * Matrix multiplication. Maybe rename this?
     * Note this type, compose arg, and return type might all be different.
     */
    public Matrix compose(Matrix other) throws Throwable {
        return new Matrix((MemorySegment) Matrix_compose.invokeExact(ptr, other.ptr));
    }

    /** Note return type may be different than this type. */
    public Matrix transpose() throws Throwable {
        return new Matrix((MemorySegment) Matrix_transpose.invokeExact(ptr));
    }

    public Vector3 times(Vector3 v) throws Throwable {
        return new Vector3((MemorySegment) Matrix_timesVector3.invokeExact(ptr, v.ptr));
    }

}
