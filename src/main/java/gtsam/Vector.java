package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * See gtsam/base/Vector.h
 * 
 * {@snippet :
 * typedef Eigen::VectorXd Vector;
 * which is Matrix<double, Dynamic, 1>
 * }
 */
public class Vector extends ForeignObject
        implements VectorType<Vector>, Manifold<Vector, Vector> {
    public enum FF {
        Vector(ADDRESS, JAVA_INT),
        Vector_delete(null, ADDRESS),
        Vector_set(null, ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector_minus(ADDRESS, ADDRESS, ADDRESS),
        Vector_plus(ADDRESS, ADDRESS, ADDRESS),
        Vector_times(ADDRESS, ADDRESS, JAVA_DOUBLE),
        Vector_transpose(ADDRESS, ADDRESS),
        Vector_fromVector2(ADDRESS, ADDRESS),
        Vector_fromVector3(ADDRESS, ADDRESS),
        Vector_fromVector6(ADDRESS, ADDRESS),
        Vector_Local(ADDRESS, ADDRESS, ADDRESS),
        Vector_rows(JAVA_INT, ADDRESS),
        Vector_at(JAVA_DOUBLE, ADDRESS, JAVA_INT),
        Vector_Constant(ADDRESS, JAVA_INT, JAVA_DOUBLE),
        Vector_LinSpaced(ADDRESS, JAVA_INT, JAVA_DOUBLE, JAVA_DOUBLE),
        Vector_print(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    @Override
    public Vector dxZero() {
        return new Vector(dimension());
    }

    public Vector(MemorySegment p) {
        super(p, FF.Vector_delete.h);
    }

    public Vector(int size) {
        this((MemorySegment) FF.Vector.invoke(size));
    }

    public Vector(Vector2 v) {
        this((MemorySegment) FF.Vector_fromVector2.invoke(v.ptr));
    }

    public Vector(Vector3 v) {
        this((MemorySegment) FF.Vector_fromVector3.invoke(v.ptr));
    }

    public Vector(Vector6 v) {
        this((MemorySegment) FF.Vector_fromVector6.invoke(v.ptr));
    }

    public Vector(double[] vals) {
        this(vals.length);
        for (int i = 0; i < vals.length; ++i) {
            set(i, vals[i]);
        }
    }

    public static Vector Constant(int size, double value) {
        return new Vector((MemorySegment) FF.Vector_Constant.invoke(size, value));
    }

    public static Vector LinSpaced(int size, double low, double high) {
        return new Vector((MemorySegment) FF.Vector_LinSpaced.invoke(size, low, high));
    }

    public Vector local(Vector other) {
        return other.minus(this);
    }

    @Override
    public void set(int i, double val) {
        FF.Vector_set.invoke(ptr, i, val);
    }

    @Override
    public Vector minus(Vector other) {
        return new Vector((MemorySegment) FF.Vector_minus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector plus(Vector other) {
        return new Vector((MemorySegment) FF.Vector_plus.invoke(ptr, other.ptr));
    }

    @Override
    public Vector times(double a) {
        return new Vector((MemorySegment) FF.Vector_times.invoke(ptr, a));
    }

    /**
     * makes a row vector, represented as a matrix (since there is no row vector
     * object)
     */
    public Matrix transpose() {
        return new Matrix((MemorySegment) FF.Vector_transpose.invoke(ptr));
    }

    public int rows() {
        return (int) FF.Vector_rows.invoke(ptr);
    }

    @Override
    public double at(int i) {
        return (double) FF.Vector_at.invoke(ptr, i);
    }

    @Override
    public String toString() {
        StringBuilder b = new StringBuilder();
        int rows = rows();
        b.append("[");
        for (int r = 0; r < rows; ++r) {
            b.append(String.format(" %8.5f ", at(r)));
        }
        b.append("]\n");
        return b.toString();
    }

    @Override
    public int dimension() {
        return rows();
    }

    public Vector retract(Vector v) {
        return plus(v);
    }

    @Override
    public Vector local(Vector g, Matrix H1, Matrix H2) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Vector retract(Vector v, Matrix H1, Matrix H2) {
        throw new UnsupportedOperationException();
    }

    public void print(String label) {
        System.out.println(label);
        FF.Vector_print.invoke(ptr);
    }
}
