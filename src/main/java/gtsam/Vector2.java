package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector2 extends ForeignObject implements Manifold<Vector2, Vector2> {
    private static final MethodHandle Vector2 = Lib.down(
            "Vector2", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Vector2_delete = Lib.downVoid(
            "Vector2_delete", ADDRESS);
    private static final MethodHandle Vector2_minus = Lib.down(
            "Vector2_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector2_plus = Lib.down(
            "Vector2_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector2_at = Lib.down(
            "Vector2_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector2_print = Lib.downVoid(
            "Vector2_print", ADDRESS);

    public static class Traits implements Manifold.Traits<Vector2, Vector2> {

    }

    public static final Traits traits = new Traits();

    @Override
    public Traits traits() {
        return traits;
    }

    public Vector2(MemorySegment p) {
        super(p, Vector2_delete);
    }

    public Vector2(double v0, double v1) throws Throwable {
        this((MemorySegment) Vector2.invokeExact(v0, v1));
    }

    public double at(int i) throws Throwable {
        return (double) Vector2_at.invokeExact(ptr, i);
    }

    public void print() throws Throwable {
        Vector2_print.invokeExact(ptr);
    }

    @Override
    public int dimension() {
        return 2;
    }

    // TODO: maybe make a real "local" ?
    @Override
    public Vector2 local(Vector2 other) throws Throwable {
        return new Vector2((MemorySegment) Vector2_minus.invokeExact(other.ptr, ptr));
    }

    public Vector2 plus(Vector2 other) throws Throwable {
        return new Vector2((MemorySegment) Vector2_plus.invokeExact(ptr, other.ptr));
    }

    public boolean equals(double[] x, double tol) throws Throwable {
        int rows = x.length;
        if (rows != 2)
            return false;
        for (int i = 0; i < rows; ++i) {
            if (Math.abs(x[i] - at(i)) > tol)
                return false;
        }
        return true;
    }

    @Override
    public Vector2 retract(Vector2 v) throws Throwable {
        return new Vector2((MemorySegment) Vector2_plus.invokeExact(ptr, v.ptr));
    }

}
