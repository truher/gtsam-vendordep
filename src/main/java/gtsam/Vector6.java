package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

// TODO: finish implementation
public class Vector6 extends ForeignObject implements VectorType<Vector6> {
    private static final MethodHandle Vector6 = Lib.down(
            "Vector6", ADDRESS);
    private static final MethodHandle Vector6_delete = Lib.downVoid(
            "Vector6_delete", ADDRESS);
    private static final MethodHandle Vector6_at = Lib.down(
            "Vector6_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector6_set = Lib.downVoid(
            "Vector6_set", ADDRESS, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Vector6_plus = Lib.down(
            "Vector6_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector6_minus = Lib.down(
            "Vector6_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector6_times = Lib.down(
            "Vector6_times", ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Vector6(MemorySegment p) {
        super(p, Vector6_delete);
    }

    public Vector6() throws Throwable {
        this((MemorySegment) Vector6.invokeExact());
    }

    public Vector6(//
            double v0, double v1, double v2, //
            double v3, double v4, double v5) throws Throwable {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
        set(3, v3);
        set(4, v4);
        set(5, v5);
    }

    @Override
    public int dimension() throws Throwable {
        return 6;
    }

    @Override
    public double at(int i) throws Throwable {
        return (double) Vector6_at.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        Vector6_set.invokeExact(ptr, i, val);
    }

    @Override
    public Vector6 plus(Vector6 other) throws Throwable {
        return new Vector6((MemorySegment) Vector6_plus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector6 minus(Vector6 other) throws Throwable {
        return new Vector6((MemorySegment) Vector6_minus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector6 times(double a) throws Throwable {
        return new Vector6((MemorySegment) Vector6_times.invokeExact(ptr, a));
    }

}
