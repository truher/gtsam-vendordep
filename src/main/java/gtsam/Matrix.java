package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Matrix extends ForeignObject {
    private static final MethodHandle Matrix = Lib.down(
            "Matrix", ADDRESS);
    private static final MethodHandle Matrix_withRowsCols = Lib.down(
            "Matrix_withRowsCols", ADDRESS, JAVA_INT, JAVA_INT);
    private static final MethodHandle Matrix_delete = Lib.downVoid(
            "Matrix_delete", ADDRESS);
    private static final MethodHandle Matrix_Matrix3 = Lib.down(
            "Matrix_Matrix3", ADDRESS, ADDRESS);
    private static final MethodHandle Matrix_set = Lib.downVoid(
            "Matrix_set", ADDRESS, JAVA_INT, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Matrix_setCol = Lib.downVoid(
            "Matrix_setCol", ADDRESS, JAVA_INT, ADDRESS);
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
    private static final MethodHandle Matrix_equals = Lib.down(
            "Matrix_equals", JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Matrix(MemorySegment p) {
        super(p, Matrix_delete);
    }

    public Matrix() throws Throwable {
        this((MemorySegment) Matrix.invokeExact());
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

    public void setCol(int col, Vector v) throws Throwable {
        Matrix_setCol.invokeExact(ptr, col, v.ptr);
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

    @Override
    public boolean equals(Object obj) {
        try {
            if (obj == this)
                return true;
            if (!(obj instanceof Matrix))
                return false;
            Matrix other = (Matrix) obj;
            if (rows() != other.rows())
                return false;
            if (cols() != other.cols())
                return false;
            for (int row = 0; row < rows(); ++row) {
                for (int col = 0; col < cols(); ++col) {
                    if (Math.abs(at(row, col) - other.at(row, col)) > 5e-6)
                        return false;
                }
            }
            return true;
        } catch (Throwable e) {
            return false;
        }
    }

    /** Row-major */
    public boolean equals(double[][] x, double tol) throws Throwable {
        int rows = x.length;
        if (rows != rows())
            return false;
        for (int i = 0; i < rows; ++i) {
            int cols = x[i].length;
            if (cols != cols())
                return false;
            for (int j = 0; j < cols; ++j) {
                if (Math.abs(x[i][j] - at(i, j)) > tol)
                    return false;
            }
        }
        return true;
    }

    public boolean equals(Matrix other, double tol) throws Throwable {
        return (boolean) Matrix_equals.invokeExact(ptr, other.ptr, tol);
    }

    public Matrix inverse() throws Throwable {
        return new Matrix((MemorySegment) Matrix_inverse.invokeExact(ptr));
    }

    public Matrix compose(Matrix other) throws Throwable {
        return new Matrix((MemorySegment) Matrix_compose.invokeExact(ptr, other.ptr));
    }

    public Matrix transpose() throws Throwable {
        return new Matrix((MemorySegment) Matrix_transpose.invokeExact(ptr));
    }

}
