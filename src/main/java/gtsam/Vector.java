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
 * typedef Eigen::VectorXd Vector;
 * which is Matrix<double, Dynamic, 1>
 */
public class Vector extends ForeignObject {
    private static final MethodHandle Vector = Lib.down(
            "Vector", ADDRESS, JAVA_INT);
    private static final MethodHandle Vector_delete = Lib.downVoid(
            "Vector_delete", ADDRESS);
    private static final MethodHandle Vector_set = Lib.downVoid(
            "Vector_set", ADDRESS, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Vector_fromTangentVector = Lib.down(
            "Vector_fromTangentVector", ADDRESS, ADDRESS);
    private static final MethodHandle Vector_rows = Lib.down(
            "Vector_rows", JAVA_INT, ADDRESS);
    private static final MethodHandle Vector_at = Lib.down(
            "Vector_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);

    public Vector(MemorySegment p) {
        super(p, Vector_delete);
    }

    public Vector(int size) throws Throwable {
        this((MemorySegment) Vector.invokeExact(size));
    }

    public Vector(TangentVector v) throws Throwable {
        this((MemorySegment) Vector_fromTangentVector.invokeExact(v.ptr));
    }

    public Vector(double[] vals) throws Throwable {
        this(vals.length);
        for (int i = 0; i < vals.length; ++i) {
            set(i, vals[i]);
        }
    }

    public void set(int i, double val) throws Throwable {
        Vector_set.invokeExact(ptr, i, val);
    }

    public int rows() throws Throwable {
        return (int) Vector_rows.invokeExact(ptr);
    }

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
                b.append(String.format("%6.3f", at(r)));
            }
            b.append("]\n");
            return b.toString();
        } catch (Throwable e) {
            e.printStackTrace();
            return "";
        }

    }
}
