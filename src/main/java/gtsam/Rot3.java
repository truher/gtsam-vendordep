package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Rot3 extends ForeignObject {
    private static final MethodHandle Rot3 = Lib.down(
            "Rot3", ADDRESS,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE,
            JAVA_DOUBLE, JAVA_DOUBLE, JAVA_DOUBLE);
    private static final MethodHandle Rot3_delete = Lib.downVoid(
            "Rot3_delete", ADDRESS);

    public Rot3(MemorySegment p) {
        super(p, Rot3_delete);
    }

    public Rot3( //
            double R11, double R12, double R13, //
            double R21, double R22, double R23, //
            double R31, double R32, double R33) throws Throwable {
        this((MemorySegment) Rot3.invokeExact(//
                R11, R12, R13, //
                R21, R22, R23, //
                R31, R32, R33));
    }

}
