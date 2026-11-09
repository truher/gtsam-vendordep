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
        Matrix_print(null, ADDRESS),
        Matrix_draw(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Matrix(MemorySegment p) {
        super(p, FF.Matrix_delete.h);
    }

    public Matrix() {
        this((MemorySegment) FF.Matrix.invoke());
    }

    public static Matrix I_1x1() {
        return new Matrix((MemorySegment) FF.Matrix_identity1.invoke());
    }

    public static Matrix I_2x2() {
        return new Matrix((MemorySegment) FF.Matrix_identity2.invoke());
    }

    public static Matrix I_3x3() {
        return new Matrix((MemorySegment) FF.Matrix_identity3.invoke());
    }

    public static Matrix I_4x4() {
        return new Matrix((MemorySegment) FF.Matrix_identity4.invoke());
    }

    public Matrix(int rows, int cols) {
        this((MemorySegment) FF.Matrix_withRowsCols.invoke(rows, cols));
    }

    public Matrix(Matrix3 m) {
        this((MemorySegment) FF.Matrix_Matrix3.invoke(m.ptr));
    }

    public Matrix(double[][] x) {
        int rows = x.length;
        int cols = x[0].length;
        this(rows, cols);
        for (int row = 0; row < rows(); ++row) {
            for (int col = 0; col < cols(); ++col) {
                set(row, col, x[row][col]);
            }
        }
    }

    public void set(int row, int col, double v) {
        FF.Matrix_set.invoke(ptr, row, col, v);
    }

    public <V extends VectorType<V>> void setCol(int col, V v) {
        for (int row = 0; row < v.dimension(); ++row) {
            set(row, col, v.at(row));
        }
    }

    public void set(double[][] x) {
        int rows = x.length;
        int cols = x[0].length;
        if (rows != rows())
            throw new IllegalArgumentException();
        if (cols != cols())
            throw new IllegalArgumentException();
        for (int row = 0; row < rows(); ++row) {
            for (int col = 0; col < cols(); ++col) {
                set(row, col, x[row][col]);
            }
        }
    }

    public double at(int r, int c) {
        return (double) FF.Matrix_at.invoke(ptr, r, c);
    }

    public Vector diagonal_cwiseSqrt() {
        return new Vector(
                (MemorySegment) FF.Matrix_diagonal_cwiseSqrt.invoke(ptr));
    }

    public int rows() {
        return (int) FF.Matrix_rows.invoke(ptr);
    }

    public int cols() {
        return (int) FF.Matrix_cols.invoke(ptr);
    }

    /** TODO: what if not square? Pseudo-inverse? */
    public Matrix inverse() {
        return new Matrix((MemorySegment) FF.Matrix_inverse.invoke(ptr));
    }

    /**
     * Matrix multiplication. Maybe rename this?
     * Note this type, compose arg, and return type might all be different.
     */
    public Matrix compose(Matrix other) {
        return new Matrix((MemorySegment) FF.Matrix_compose.invoke(ptr, other.ptr));
    }

    /** Note return type may be different than this type. */
    public Matrix transpose() {
        return new Matrix((MemorySegment) FF.Matrix_transpose.invoke(ptr));
    }

    public Vector times(Vector v) {
        return new Vector((MemorySegment) FF.Matrix_timesVector.invoke(ptr, v.ptr));
    }

    public Vector3 times(Vector3 v) {
        return new Vector3((MemorySegment) FF.Matrix_timesVector3.invoke(ptr, v.ptr));
    }

    public Matrix times(double a) {
        return new Matrix((MemorySegment) FF.Matrix_timesDouble.invoke(ptr, a));
    }

    public static boolean linear_dependent(Matrix A, Matrix B) {
        return (boolean) FF.Matrix_linear_dependent.invoke(A.ptr, B.ptr, 1e-9);
    }

    public static boolean linear_dependent(Matrix A, Matrix B, double tol) {
        return (boolean) FF.Matrix_linear_dependent.invoke(A.ptr, B.ptr, tol);
    }

    public void print(String label) {
        System.out.println(label);
        FF.Matrix_print.invoke(ptr);
    }

    /**
     * If the matrix is the covariance matrix of an estimate, then this function
     * produces a sample from the multivariate normal distribution described by the
     * covariance matrix.
     */
    public Vector draw() {
        return new Vector((MemorySegment) FF.Matrix_draw.invoke(ptr));
    }
}
