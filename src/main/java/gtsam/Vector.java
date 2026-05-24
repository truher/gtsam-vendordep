package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
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
    private static final MethodHandle Vector = Lib.down(
            "Vector", ADDRESS, JAVA_INT);
    private static final MethodHandle Vector_delete = Lib.downVoid(
            "Vector_delete", ADDRESS);
    private static final MethodHandle Vector_set = Lib.downVoid(
            "Vector_set", ADDRESS, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Vector_minus = Lib.down(
            "Vector_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector_plus = Lib.down(
            "Vector_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector_times = Lib.down(
            "Vector_times", ADDRESS, ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Vector_fromTangentVector = Lib.down(
            "Vector_fromTangentVector", ADDRESS, ADDRESS);
    private static final MethodHandle Vector_fromVector2 = Lib.down(
            "Vector_fromVector2", ADDRESS, ADDRESS);
    private static final MethodHandle Vector_fromVector3 = Lib.down(
            "Vector_fromVector3", ADDRESS, ADDRESS);
    private static final MethodHandle Vector_Local = Lib.down(
            "Vector_Local", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector_rows = Lib.down(
            "Vector_rows", JAVA_INT, ADDRESS);
    private static final MethodHandle Vector_at = Lib.down(
            "Vector_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);

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
        super(p, Vector_delete);
    }

    public Vector(int size) throws Throwable {
        this((MemorySegment) Vector.invokeExact(size));
    }

    public Vector(TangentVector v) throws Throwable {
        this((MemorySegment) Vector_fromTangentVector.invokeExact(v.ptr));
    }

    public Vector(Vector2 v) throws Throwable {
        this((MemorySegment) Vector_fromVector2.invokeExact(v.ptr));
    }

    public Vector(Vector3 v) throws Throwable {
        this((MemorySegment) Vector_fromVector3.invokeExact(v.ptr));
    }

    public Vector(double[] vals) throws Throwable {
        this(vals.length);
        for (int i = 0; i < vals.length; ++i) {
            set(i, vals[i]);
        }
    }

    // TODO: maybe use "minus" insteaad?
    public Vector local(Vector other) throws Throwable {
        return new Vector((MemorySegment) Vector_Local.invokeExact(ptr, other.ptr));
    }

    @Override
    public void set(int i, double val) throws Throwable {
        Vector_set.invokeExact(ptr, i, val);
    }

    @Override
    public Vector minus(Vector other) throws Throwable {
        return new Vector((MemorySegment) Vector_minus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector plus(Vector other) throws Throwable {
        return new Vector((MemorySegment) Vector_plus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector times(double a) throws Throwable {
        return new Vector((MemorySegment) Vector_times.invokeExact(ptr, a));
    }

    public int rows() throws Throwable {
        return (int) Vector_rows.invokeExact(ptr);
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) Vector_at.invokeExact(ptr, i);
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
