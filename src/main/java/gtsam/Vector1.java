package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector1 extends ForeignObject
        implements VectorType<Vector1> {
    private static final MethodHandle Vector1 = Lib.down(
            "Vector1", ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Vector1_delete = Lib.downVoid(
            "Vector1_delete", ADDRESS);

    private static final MethodHandle Vector1_at = Lib.down(
            "Vector1_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector1_set = Lib.downVoid(
            "Vector1_set", ADDRESS, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Vector1_plus = Lib.down(
            "Vector1_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector1_minus = Lib.down(
            "Vector1_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector1_times = Lib.down(
            "Vector1_times", ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Vector1(MemorySegment p) {
        super(p, Vector1_delete);
    }

    public Vector1(double v0) throws Throwable {
        this((MemorySegment) Vector1.invokeExact(v0));
    }

    @Override
    public int dimension() throws Throwable {
        return 1;
    }

   @Override
    public double at(int i) throws Throwable {
        return (double) Vector1_at.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        Vector1_set.invokeExact(ptr, i, val);
    }

    @Override
    public Vector1 plus(Vector1 other) throws Throwable {
        return new Vector1((MemorySegment) Vector1_plus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector1 minus(Vector1 other) throws Throwable {
        return new Vector1((MemorySegment) Vector1_minus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector1 times(double a) throws Throwable {
        return new Vector1((MemorySegment) Vector1_times.invokeExact(ptr, a));
    }
}
