package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector3 extends ForeignObject {
    private static final MethodHandle Vector3 = Lib.down(
            "Vector3", ADDRESS, JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Vector3_delete = Lib.downVoid(
            "Vector3_delete", ADDRESS);
    private static final MethodHandle Vector3_at = Lib.down(
            "Vector3_at", JAVA_DOUBLE, ADDRESS, JAVA_INT);
    private static final MethodHandle Vector3_print = Lib.downVoid(
            "Vector3_print", ADDRESS);

    public Vector3(MemorySegment p) {
        super(p, Vector3_delete);
    }

    public Vector3(double v0, double v1, double v2) throws Throwable {
        this((MemorySegment) Vector3.invokeExact(v0, v1, v2));
    }

    public double at(int i) throws Throwable {
        return (double) Vector3_at.invokeExact(ptr, i);
    }

    public void print() throws Throwable {
        Vector3_print.invokeExact(ptr);
    }

}
