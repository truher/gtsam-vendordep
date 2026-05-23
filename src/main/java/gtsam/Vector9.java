package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

// TODO: finish implementation
public class Vector9 extends ForeignObject implements VectorType<Vector9> {
    private static final MethodHandle Vector9 = Lib.down(
            "Vector9", ADDRESS);
    private static final MethodHandle Vector9_delete = Lib.downVoid(
            "Vector9_delete", ADDRESS);
    private static final MethodHandle Vector9_at = Lib.down(
            "Vector9_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector9_set = Lib.downVoid(
            "Vector9_set", ADDRESS, JAVA_INT, JAVA_DOUBLE);
    private static final MethodHandle Vector9_plus = Lib.down(
            "Vector9_plus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector9_minus = Lib.down(
            "Vector9_minus", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Vector9_times = Lib.down(
            "Vector9_times", ADDRESS, ADDRESS, JAVA_DOUBLE);

    public Vector9(MemorySegment p) {
        super(p, Vector9_delete);
    }

    public Vector9() throws Throwable {
        this((MemorySegment) Vector9.invokeExact());
    }

    public Vector9(//
            double v0, double v1, double v2, //
            double v3, double v4, double v5, //
            double v6, double v7, double v8) throws Throwable {
        this();
        set(0, v0);
        set(1, v1);
        set(2, v2);
        set(3, v3);
        set(4, v4);
        set(5, v5);
        set(6, v6);
        set(7, v7);
        set(8, v8);
    }

    @Override
    public int dimension() throws Throwable {
        return 9;
    }

     @Override
    public double at(int i) throws Throwable {
        return (double) Vector9_at.invokeExact(ptr, i);
    }

    @Override
    public void set(int i, double val) throws Throwable {
        Vector9_set.invokeExact(ptr, i, val);
    }

    @Override
    public Vector9 plus(Vector9 other) throws Throwable {
        return new Vector9((MemorySegment) Vector9_plus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector9 minus(Vector9 other) throws Throwable {
        return new Vector9((MemorySegment) Vector9_minus.invokeExact(ptr, other.ptr));
    }

    @Override
    public Vector9 times(double a) throws Throwable {
        return new Vector9((MemorySegment) Vector9_times.invokeExact(ptr, a));
    }

}
