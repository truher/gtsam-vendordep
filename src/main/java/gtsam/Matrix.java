package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * Dynamically-dimensioned Matrix, corresponds to Eigen::MatrixXd.
 */
public class Matrix extends ForeignObject {
    public enum FF {
        Matrix(ADDRESS),
        Matrix_identity1(ADDRESS),
        Matrix_identity2(ADDRESS),
        Matrix_identity3(ADDRESS),
        Matrix_identity4(ADDRESS),
        Matrix_withRowsCols(ADDRESS, JAVA_INT, JAVA_INT),
        Matrix_delete(null, ADDRESS),
        Matrix_Matrix3(ADDRESS, ADDRESS),
        Matrix_set(null, ADDRESS, JAVA_INT, JAVA_INT, JAVA_DOUBLE),
        Matrix_at(JAVA_DOUBLE, ADDRESS, JAVA_INT, JAVA_INT),
        Matrix_diagonal_cwiseSqrt(ADDRESS, ADDRESS),
        Matrix_rows(JAVA_INT, ADDRESS),
        Matrix_cols(JAVA_INT, ADDRESS),
        Matrix_inverse(ADDRESS, ADDRESS),
        Matrix_compose(ADDRESS, ADDRESS, ADDRESS),
        Matrix_transpose(ADDRESS, ADDRESS),
        Matrix_timesVector(ADDRESS, ADDRESS, ADDRESS),
        Matrix_timesVector3(ADDRESS, ADDRESS, ADDRESS),
        Matrix_timesDouble(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Matrix_linear_dependent(JAVA_BOOLEAN, ADDRESS, ADDRESS, JAVA_DOUBLE),
        Matrix_print(null, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Matrix(MemorySegment p) {
        super(p, FF.Matrix_delete.h);
    }

    public Matrix() throws Throwable {
        this((MemorySegment) FF.Matrix.h.invokeExact());
    }

    public static Matrix I_1x1() throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_identity1.h.invokeExact());
    }

    public static Matrix I_2x2() throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_identity2.h.invokeExact());
    }

    public static Matrix I_3x3() throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_identity3.h.invokeExact());
    }

    public static Matrix I_4x4() throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_identity4.h.invokeExact());
    }

    public Matrix(int rows, int cols) throws Throwable {
        this((MemorySegment) FF.Matrix_withRowsCols.h.invokeExact(rows, cols));
    }

    public Matrix(Matrix3 m) throws Throwable {
        this((MemorySegment) FF.Matrix_Matrix3.h.invokeExact(m.ptr));
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
        FF.Matrix_set.h.invokeExact(ptr, row, col, v);
    }

    public <V extends VectorType<V>> void setCol(int col, V v) throws Throwable {
        for (int row = 0; row < v.dimension(); ++row) {
            set(row, col, v.at(row));
        }
    }

    public double at(int r, int c) throws Throwable {
        return (double) FF.Matrix_at.h.invokeExact(ptr, r, c);
    }

    public Vector diagonal_cwiseSqrt() throws Throwable {
        return new Vector(
                (MemorySegment) FF.Matrix_diagonal_cwiseSqrt.h.invokeExact(ptr));
    }

    public int rows() throws Throwable {
        return (int) FF.Matrix_rows.h.invokeExact(ptr);
    }

    public int cols() throws Throwable {
        return (int) FF.Matrix_cols.h.invokeExact(ptr);
    }

    /** TODO: what if not square? Pseudo-inverse? */
    public Matrix inverse() throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_inverse.h.invokeExact(ptr));
    }

    /**
     * Matrix multiplication. Maybe rename this?
     * Note this type, compose arg, and return type might all be different.
     */
    public Matrix compose(Matrix other) throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_compose.h.invokeExact(ptr, other.ptr));
    }

    /** Note return type may be different than this type. */
    public Matrix transpose() throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_transpose.h.invokeExact(ptr));
    }

    public Vector times(Vector v) throws Throwable {
        return new Vector((MemorySegment) FF.Matrix_timesVector.h.invokeExact(ptr, v.ptr));
    }

    public Vector3 times(Vector3 v) throws Throwable {
        return new Vector3((MemorySegment) FF.Matrix_timesVector3.h.invokeExact(ptr, v.ptr));
    }

    public Matrix times(double a) throws Throwable {
        return new Matrix((MemorySegment) FF.Matrix_timesDouble.h.invokeExact(ptr, a));
    }

    public static boolean linear_dependent(Matrix A, Matrix B) throws Throwable {
        return (boolean) FF.Matrix_linear_dependent.h.invokeExact(A.ptr, B.ptr, 1e-9);
    }

    public static boolean linear_dependent(Matrix A, Matrix B, double tol) throws Throwable {
        return (boolean) FF.Matrix_linear_dependent.h.invokeExact(A.ptr, B.ptr, tol);
    }

    public void print(String label) throws Throwable {
        System.out.println(label);
        FF.Matrix_print.h.invokeExact(ptr);
    }
}
