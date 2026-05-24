package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector3 extends ForeignObject
        implements VectorType<Vector3>, Manifold<Vector3, Vector3> {
    private static final MethodHandle Vector3 = Lib.down(
            "Vector3", ADDRESS);
    private static final MethodHandle Vector3_delete = Lib.downVoid(
            "Vector3_delete", ADDRESS);
    private static final MethodHandle Vector3_at = Lib.down(
            "Vector3_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector3_set = Lib.downVoid(
            "Vector3_set", ADDRESS, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Vector3_plus = Lib.down(
            "Vector3_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector3_minus = Lib.down(
            "Vector3_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector3_times = Lib.down(
            "Vector3_times", ADDRESS, ADDRESS, JAVA_DOUBLE);

    public static class Traits implements Manifold.Traits<Vector3, Vector3> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    @Override
    public Vector3 dxZero() throws Throwable {
        return new Vector3(0, 0, 0);
    }

    public Vector3(MemorySegment p) {
        super(p, Vector3_delete);
    }

    public Vector3() throws Throwable {
        this((MemorySegment) Vector3.invokeExact());
    }

    public Vector3(double v0, double v1, double v2) throws Throwable {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
    }

    @Override
    public int dimension() throws Throwable {
        return 3;
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) Vector3_at.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        Vector3_set.invokeExact(ptr, i, val);
    }

    @Override
    public Vector3 plus(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) Vector3_plus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector3 minus(Vector3 other) throws Throwable {
        return new Vector3((MemorySegment) Vector3_minus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector3 times(double a) throws Throwable {
        return new Vector3((MemorySegment) Vector3_times.invokeExact(ptr, a));
    }

    @Override
    public String toString() {
        try {
            StringBuilder b = new StringBuilder();
            int rows = 3;
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
    public Vector3 local(Vector3 other) throws Throwable {
        return other.minus(this);
    }

    @Override
    public Vector3 retract(Vector3 v) throws Throwable {
        return plus(v);
    }
}
