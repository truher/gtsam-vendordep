package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector2 extends ForeignObject {
    private static final MethodHandle Vector2 = Lib.down(
            "Vector2", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Vector2_delete = Lib.downVoid(
            "Vector2_delete", ADDRESS);
    private static final MethodHandle Vector2_at = Lib.down(
            "Vector2_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector2_print = Lib.downVoid(
            "Vector2_print", ADDRESS);

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

}
