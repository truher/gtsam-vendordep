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
        Vector_fromVector2(ADDRESS, ADDRESS),
        Vector_fromVector3(ADDRESS, ADDRESS),
        Vector_fromVector6(ADDRESS, ADDRESS),
        Vector_Local(ADDRESS, ADDRESS, ADDRESS),
        Vector_rows(JAVA_INT, ADDRESS),
        Vector_at(JAVA_DOUBLE, ADDRESS, JAVA_INT);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public static class Traits implements Manifold.Traits<Vector, Vector> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector dxZero() throws Throwable {
        return new Vector(dimension());
    }

    public Vector(MemorySegment p) {
        super(p, FF.Vector_delete.h);
    }

    public Vector(int size) throws Throwable {
        this((MemorySegment) FF.Vector.h.invokeExact(size));
    }

    public Vector(Vector2 v) throws Throwable {
        this((MemorySegment) FF.Vector_fromVector2.h.invokeExact(v.ptr));
    }

    public Vector(Vector3 v) throws Throwable {
        this((MemorySegment) FF.Vector_fromVector3.h.invokeExact(v.ptr));
    }

    public Vector(Vector6 v) throws Throwable {
        this((MemorySegment) FF.Vector_fromVector6.h.invokeExact(v.ptr));
    }

    public Vector(double[] vals) throws Throwable {
        this(vals.length);
        for (int i = 0; i < vals.length; ++i) {
            set(i, vals[i]);
        }
    }

    public Vector localCoordinates(Vector other) throws Throwable {
        return other.minus(this);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        FF.Vector_set.h.invokeExact(ptr, i, val);
    }

    @Override
    public Vector minus(Vector other) throws Throwable {
        return new Vector((MemorySegment) FF.Vector_minus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector plus(Vector other) throws Throwable {
        return new Vector((MemorySegment) FF.Vector_plus.h.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector times(double a) throws Throwable {
        return new Vector((MemorySegment) FF.Vector_times.h.invokeExact(ptr, a));
    }

    public int rows() throws Throwable {
        return (int) FF.Vector_rows.h.invokeExact(ptr);
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) FF.Vector_at.h.invokeExact(ptr, i);
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = rows();
            b.append("[");
            for (int r = 0; r < rows; ++r) {
                b.append(String.format(" %8.5f ", at(r)));
            }
            b.append("]\n");
            return b.toString();
        } catch (Throwable e) {
            e.printStackTrace();
            return "";
        }
    }

    @Override
    public int dimension() throws Throwable {
        return rows();
    }

    public Vector retract(Vector v) throws Throwable {
        return plus(v);
    }
}
