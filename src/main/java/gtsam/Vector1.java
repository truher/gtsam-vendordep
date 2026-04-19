package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Vector1 extends ForeignObject {
    private static final MethodHandle Vector1 = Lib.down(
            "Vector1", ADDRESS, JAVA_DOUBLE);
    private static final MethodHandle Vector1_delete = Lib.downVoid(
            "Vector1_delete", ADDRESS);

    public Vector1(MemorySegment p) {
        super(p, Vector1_delete);
    }

    public Vector1(double v0) throws Throwable {
        this((MemorySegment) Vector1.invokeExact(v0));
    }
}
